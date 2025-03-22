#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/usb.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>

#define NAME "cgscreen"
#define WIDTH 128
#define HEIGHT 64
#define SCREEN_SIZE (WIDTH * HEIGHT / 8UL)
#define URB_RX (6 + SCREEN_SIZE + 2)
#define URB_BLOCKSIZE (512)
#define URB_BUFSIZE (((URB_RX - 1) / URB_BLOCKSIZE + 1) * URB_BLOCKSIZE)
#define VIDEO_SIZE (SCREEN_SIZE * 8)
#define BUFC 2
#define FOURCC V4L2_PIX_FMT_GREY

#define hextoint(hexchar) ((hexchar) <= '9' ? (hexchar) - '0' : (hexchar) + 10 - 'A')

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Baltazár Radics");
MODULE_DESCRIPTION("Casio graphing calculator screen projector USB driver");
MODULE_VERSION("0.1.0");

static const struct usb_device_id cgscreen_id_table[] = {
		{USB_DEVICE(0x07cf, 0x6101)}, // A caiso calculator in 'Projector' mode
		{}};
MODULE_DEVICE_TABLE(usb, cgscreen_id_table);

struct cgscreen_dev {
	struct kref kref;
	struct usb_device *udev;         // the usb device for this device
	struct usb_interface *intf;      // the interface for this device
	struct urb *urb;                 // the urb to read data with
	uint8_t bulk_in_endp;            // the endpoint pipe for reading
	uint8_t *urb_buf;                // the buffer to receive data
	uint8_t *done_buf;               // a complete frame
	uint32_t done_off;               // where the data starts in done_buf
	uint64_t done_seq;               // sequence number of done_buf
	ktime_t done_timestamp;          // when the last buffer was read
	wait_queue_head_t queue;         // woken up after successful read, locked during buffer swap
	uint8_t buffers[2][URB_BUFSIZE]; // the buffers themselves
	struct v4l2_device v4l2_dev;     // video4linux device
	struct video_device *vdev;       // video device
	uint8_t stream_on : 1;           // whether streaming (and the urb) is running
};

static void cgscreen_delete_dev(struct kref *kref) {
	struct cgscreen_dev *dev = container_of(kref, struct cgscreen_dev, kref);

	usb_free_urb(dev->urb);
	usb_put_intf(dev->intf);
	usb_put_dev(dev->udev);
	kfree(dev);
}

static void cgscreen_put_dev(struct cgscreen_dev *dev) {
	kref_put(&dev->kref, cgscreen_delete_dev);
}

typedef uint8_t videobuf_t[VIDEO_SIZE];

struct cgscreen_opener {
	uint32_t read_seq;       // sequence number of last read buffer
	uint32_t bufcount;       // the number of video buffers in use
	videobuf_t *buffers;     // video buffers used for streaming
	uint32_t bufinfo[BUFC];  // videobuf flags
	wait_queue_head_t queue; // woken up when a new buffer is added, locked during buffer access
	uint32_t period_ms;      // time between successive frames (in milliseconds)
	ktime_t read_timestamp;  // when the last buffer was read
};

static void cgscreen_delete_opener(struct cgscreen_opener *opener) {
	if (opener->buffers) {
		vfree(opener->buffers);
		opener->buffers = NULL;
	}
	kfree(opener);
}

static void cgscreen_vm_open(struct vm_area_struct *vma) {
	struct cgscreen_opener *opener = vma->vm_private_data;
	spin_lock(&opener->queue.lock);
	opener->bufinfo[vma->vm_pgoff / (VIDEO_SIZE >> PAGE_SHIFT)] |= V4L2_BUF_FLAG_MAPPED;
	spin_unlock(&opener->queue.lock);
}

static void cgscreen_vm_close(struct vm_area_struct *vma) {
	struct cgscreen_opener *opener = vma->vm_private_data;
	spin_lock(&opener->queue.lock);
	opener->bufinfo[vma->vm_pgoff / (VIDEO_SIZE >> PAGE_SHIFT)] &= ~V4L2_BUF_FLAG_MAPPED;
	spin_unlock(&opener->queue.lock);
}

static struct vm_operations_struct cgscreen_vmops = {
		.open = cgscreen_vm_open,
		.close = cgscreen_vm_close,
};

static int cgscreen_open(struct file *file) {
	struct cgscreen_dev *dev;
	struct cgscreen_opener *opener;

	dev = video_drvdata(file);
	if (!dev) {
		return -ENODEV;
	}

	opener = kzalloc(sizeof(*opener), GFP_KERNEL);
	if (!opener) {
		return -ENOMEM;
	}
	init_waitqueue_head(&opener->queue);
	opener->period_ms = 33; // default: ~30fps

	// increment our usage count for the device
	kref_get(&dev->kref);

	file->private_data = opener;

	return 0;
}

static int cgscreen_release(struct file *file) {
	struct cgscreen_dev *dev = video_drvdata(file);
	struct cgscreen_opener *opener = file->private_data;

	if (opener)
		cgscreen_delete_opener(opener);

	if (dev)
		cgscreen_put_dev(dev);

	return 0;
}

static __poll_t cgscreen_poll(struct file *file, struct poll_table_struct *pts) {
	struct cgscreen_opener *opener = file->private_data;
	struct cgscreen_dev *dev = video_drvdata(file);
	int done_seq, read_seq;

	spin_lock_irq(&dev->queue.lock);
	done_seq = dev->done_seq;
	spin_unlock_irq(&dev->queue.lock);

	spin_lock(&opener->queue.lock);
	read_seq = opener->read_seq;
	spin_unlock(&opener->queue.lock);

	if (read_seq < done_seq)
		return POLLIN | POLLRDNORM;

	return 0;
}

static int cgscreen_mmap(struct file *file, struct vm_area_struct *vma) {
	struct cgscreen_opener *opener = file->private_data;
	unsigned long start;
	unsigned bufnr;
	void *addr;

	if (!opener)
		return -ENODEV;
	if (vma->vm_end - vma->vm_start > VIDEO_SIZE)
		return -EINVAL;
	if (vma->vm_pgoff % (VIDEO_SIZE >> PAGE_SHIFT) != 0)
		return -EINVAL;
	bufnr = vma->vm_pgoff / (VIDEO_SIZE >> PAGE_SHIFT);
	if (bufnr >= opener->bufcount)
		return -EINVAL;

	start = vma->vm_start;
	addr = opener->buffers[bufnr];

	while (start < vma->vm_end) {
		struct page *page = vmalloc_to_page(addr);
		if (vm_insert_page(vma, start, page) < 0)
			return -EAGAIN;
		start += PAGE_SIZE;
		addr += PAGE_SIZE;
	}

	vma->vm_private_data = opener;
	vma->vm_ops = &cgscreen_vmops;
	vma->vm_ops->open(vma);

	return 0;
}

static const struct v4l2_file_operations cgscreen_fops = {
		.owner = THIS_MODULE,
		.open = cgscreen_open,
		.release = cgscreen_release,
		.poll = cgscreen_poll,
		.mmap = cgscreen_mmap,
		.unlocked_ioctl = video_ioctl2,
};

static void render01(const uint8_t *src, uint32_t off, uint8_t *dst) {
	const uint8_t *p, *end;
	uint32_t i;
	p = src + ((off + 6) % URB_BUFSIZE);
	end = src + ((off + 6 + SCREEN_SIZE) % URB_BUFSIZE);
	while (p != end) {
		for (i = 0; i < 8; ++i, ++dst)
			*dst = (*p & 1 << (7 - i)) ? 0x00 : 0xff;
		if (++p == src + URB_BUFSIZE)
			p = src;
	}
}

static int vidioc_querycap(struct file *file, void *priv, struct v4l2_capability *cap) {
	struct cgscreen_dev *dev = video_drvdata(file);

	if (!dev)
		return -ENODEV;

	strscpy(cap->driver, "cgscreen");
	strscpy(cap->card, dev->vdev->name);
	usb_make_path(dev->udev, cap->bus_info, sizeof(cap->bus_info));
	cap->version = LINUX_VERSION_CODE;
	cap->capabilities = dev->vdev->device_caps | V4L2_CAP_DEVICE_CAPS;
	cap->device_caps = dev->vdev->device_caps;
	memset(cap->reserved, 0, sizeof(cap->reserved));

	return 0;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void *fh, struct v4l2_fmtdesc *f) {
	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (f->index) // we have only one format
		return -EINVAL;

	strscpy(f->description, "8 bpp, Greyscale (GREY)");
	f->pixelformat = FOURCC;
	f->flags = 0;

	return 0;
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *fmt) {
	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	fmt->fmt.pix.width = WIDTH;
	fmt->fmt.pix.height = HEIGHT;
	fmt->fmt.pix.pixelformat = FOURCC;
	fmt->fmt.pix.field = V4L2_FIELD_NONE;
	fmt->fmt.pix.bytesperline = WIDTH;
	fmt->fmt.pix.sizeimage = VIDEO_SIZE;
	fmt->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
	fmt->fmt.pix.priv = V4L2_PIX_FMT_PRIV_MAGIC;
	fmt->fmt.pix.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->fmt.pix.quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->fmt.pix.xfer_func = V4L2_XFER_FUNC_DEFAULT;
	return 0;
}

static int vidioc_enum_input(struct file *file, void *fh, struct v4l2_input *inp) {
	if (inp->index != 0)
		return -EINVAL;

	strscpy(inp->name, "Screen");
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	inp->audioset = 0;
	inp->tuner = 0;
	inp->std = V4L2_STD_UNKNOWN;
	inp->status = 0;
	inp->capabilities = 0;
	memset(inp->reserved, 0, sizeof(inp->reserved));

	return 0;
}

static int vidioc_g_input(struct file *file, void *fh, unsigned int *i) {
	*i = 0;
	return 0;
}

static int vidioc_s_input(struct file *file, void *fh, unsigned int i) {
	if (i != 0)
		return -EINVAL;
	return 0;
}

static int vidioc_reqbufs(struct file *file, void *fh, struct v4l2_requestbuffers *b) {
	struct cgscreen_opener *opener = file->private_data;

	if (!opener)
		return -ENODEV;
	if (b->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (b->memory != V4L2_MEMORY_MMAP)
		return -EINVAL;

	spin_lock(&opener->queue.lock);

	if (b->count > 0) {
		opener->buffers = vmalloc(VIDEO_SIZE * BUFC);
		if (!opener->buffers) {
			spin_unlock(&opener->queue.lock);
			return -ENOMEM;
		}
		memset32(opener->bufinfo, V4L2_BUF_FLAG_PREPARED | V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC, BUFC);
		opener->bufcount = b->count = BUFC;
	} else {
		opener->bufcount = b->count = 0;
		kfree(opener->buffers);
		opener->buffers = NULL;
		memset32(opener->bufinfo, 0, BUFC);
	}

	spin_unlock(&opener->queue.lock);

	b->capabilities = V4L2_BUF_CAP_SUPPORTS_MMAP;

	return 0;
}

static int vidioc_querybuf(struct file *file, void *fh, struct v4l2_buffer *b) {
	struct cgscreen_opener *opener = file->private_data;

	if (!opener)
		return -ENODEV;
	if (b->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	spin_lock(&opener->queue.lock);

	if (b->index > opener->bufcount) {
		spin_unlock(&opener->queue.lock);
		return -EINVAL;
	}

	b->bytesused = VIDEO_SIZE;
	b->flags = opener->bufinfo[b->index];
	b->field = V4L2_FIELD_NONE;
	b->memory = V4L2_MEMORY_MMAP;
	b->m.offset = VIDEO_SIZE * b->index;
	b->length = VIDEO_SIZE;
	b->reserved = b->reserved2 = 0;

	spin_unlock(&opener->queue.lock);

	return 0;
}

// queue a buffer to be filled
static int vidioc_qbuf(struct file *file, void *fh, struct v4l2_buffer *b) {
	struct cgscreen_opener *opener = file->private_data;

	if (!opener)
		return -ENODEV;
	if (b->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (b->index > BUFC)
		return -EINVAL;

	spin_lock(&opener->queue.lock);

	if (!(opener->bufinfo[b->index] & V4L2_BUF_FLAG_PREPARED)) {
		spin_unlock(&opener->queue.lock);
		return -EINVAL;
	}

	b->flags = opener->bufinfo[b->index] ^= V4L2_BUF_FLAG_PREPARED | V4L2_BUF_FLAG_QUEUED;

	spin_unlock(&opener->queue.lock);
	wake_up_locked(&opener->queue);

	return 0;
}

static int findbuf(struct cgscreen_opener *opener) __must_hold(opener->queue.lock) {
	int i;
	for (i = 0; i < BUFC; ++i)
		if (opener->bufinfo[i] & V4L2_BUF_FLAG_QUEUED)
			break;
	return i;
}

static int waitbuf(struct cgscreen_opener *opener, bool nonblock) __must_hold(opener->queue.lock) {
	int buf = findbuf(opener);
	if (buf != BUFC)
		return buf;
	if (nonblock)
		return -EAGAIN;
	if (wait_event_interruptible_locked(opener->queue, ((buf = findbuf(opener)) != BUFC)))
		return -EINTR;
	return buf;
}

//static int waitseq(struct cgscreen_dev *dev, uint64_t seq, bool nonblock) __must_hold(dev->queue.lock) {
//	if (dev->done_seq > seq)
//		return dev->done_seq;
//	if (nonblock)
//		return -EAGAIN;
//	if (wait_event_interruptible_locked(dev->queue, (dev->done_seq > seq)))
//		return -EINTR;
//	return dev->done_seq;
//}

static ktime_t waitframe(ktime_t last, int64_t period_ms, bool nonblock) {
	ktime_t now = ktime_get();
	int64_t delay = ktime_to_ms(last - now) + period_ms;
	if (delay < 0)
		return now;
	if (nonblock)
		return -EAGAIN;
	if (msleep_interruptible(delay) < (uint64_t)delay)
		return -EINTR;
	return ktime_get();
}

// dequeue a filled buffer
static int vidioc_dqbuf(struct file *file, void *fh, struct v4l2_buffer *b) {
	struct cgscreen_dev *dev = video_drvdata(file);
	struct cgscreen_opener *opener = file->private_data;
	int seq, buf;
	ktime_t timestamp;
	const bool nonblock = file->f_flags & O_NONBLOCK;

	if (!dev || !opener)
		return -ENODEV;
	if (b->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	spin_lock(&opener->queue.lock);
	buf = waitbuf(opener, nonblock);
	if (buf < 0) {
		spin_unlock(&opener->queue.lock);
		return buf;
	}
	opener->bufinfo[buf] &= ~V4L2_BUF_FLAG_QUEUED;
	// seq = opener->read_seq;
	timestamp = opener->read_timestamp;
	spin_unlock(&opener->queue.lock);

	timestamp = waitframe(timestamp, opener->period_ms, nonblock);
	if (timestamp < 0) {
		opener->bufinfo[buf] |= V4L2_BUF_FLAG_QUEUED;
		return timestamp;
	}

	spin_lock_irq(&dev->queue.lock);
	// seq = waitseq(dev, seq, nonblock);
	//if (seq < 0) {
	//	spin_unlock_irq(&dev->queue.lock);
	//	opener->bufinfo[buf] |= V4L2_BUF_FLAG_QUEUED;
	//	return seq;
	//}
	seq = dev->done_seq;
	render01(dev->done_buf, dev->done_off, opener->buffers[buf]);
	spin_unlock_irq(&dev->queue.lock);

	spin_lock(&opener->queue.lock);
	opener->bufinfo[buf] |= V4L2_BUF_FLAG_PREPARED;
	opener->read_seq = seq;
	b->index = buf;
	b->bytesused = VIDEO_SIZE;
	b->flags = opener->bufinfo[buf];
	b->field = V4L2_FIELD_NONE;
	b->timestamp.tv_usec = timestamp / 1000 % 1000000;
	b->timestamp.tv_sec = timestamp / 1000 / 1000000;
	b->sequence = opener->read_seq;
	b->memory = V4L2_MEMORY_MMAP;
	b->m.offset = (VIDEO_SIZE >> PAGE_SHIFT) * b->index;
	b->length = VIDEO_SIZE;
	b->reserved = b->reserved2 = 0;
	spin_unlock(&opener->queue.lock);

	return 0;
}

static int vidioc_streamon(struct file *file, void *fh, enum v4l2_buf_type type) {
	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	return 0;
}

static int vidioc_streamoff(struct file *file, void *fh, enum v4l2_buf_type type) {
	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	return 0;
}

static int vidioc_g_parm(struct file *file, void *fh, struct v4l2_streamparm *a) {
	struct cgscreen_opener *opener = file->private_data;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	spin_lock(&opener->queue.lock);

	a->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	a->parm.capture.capturemode = 0;
	a->parm.capture.timeperframe.numerator = opener->period_ms;
	a->parm.capture.timeperframe.denominator = 1000;
	a->parm.capture.extendedmode = 0;
	a->parm.capture.readbuffers = 0;
	memset(a->parm.capture.reserved, 0, sizeof(a->parm.capture.reserved));

	spin_unlock(&opener->queue.lock);

	return 0;
}

static int vidioc_s_parm(struct file *file, void *fh, struct v4l2_streamparm *a) {
	struct cgscreen_opener *opener = file->private_data;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	spin_lock(&opener->queue.lock);

	a->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	a->parm.capture.capturemode = 0;
	if (a->parm.capture.timeperframe.denominator != 0)
		opener->period_ms =
				1000 * a->parm.capture.timeperframe.numerator / a->parm.capture.timeperframe.denominator;
	a->parm.capture.timeperframe.numerator = opener->period_ms;
	a->parm.capture.timeperframe.denominator = 1000;
	a->parm.capture.extendedmode = 0;
	a->parm.capture.readbuffers = 0;
	memset(a->parm.capture.reserved, 0, sizeof(a->parm.capture.reserved));

	spin_unlock(&opener->queue.lock);

	return 0;
}

static const struct v4l2_ioctl_ops cgscreen_ioctl_ops = {
		.vidioc_querycap = vidioc_querycap,
		.vidioc_enum_fmt_vid_cap = vidioc_enum_fmt_vid_cap,
		.vidioc_g_fmt_vid_cap = vidioc_g_fmt_vid_cap,
		.vidioc_s_fmt_vid_cap = vidioc_g_fmt_vid_cap,
		.vidioc_try_fmt_vid_cap = vidioc_g_fmt_vid_cap,
		.vidioc_enum_input = vidioc_enum_input,
		.vidioc_g_input = vidioc_g_input,
		.vidioc_s_input = vidioc_s_input,
		.vidioc_reqbufs = vidioc_reqbufs,
		.vidioc_querybuf = vidioc_querybuf,
		.vidioc_qbuf = vidioc_qbuf,
		.vidioc_dqbuf = vidioc_dqbuf,
		.vidioc_streamon = vidioc_streamon,
		.vidioc_streamoff = vidioc_streamoff,
		.vidioc_g_parm = vidioc_g_parm,
		.vidioc_s_parm = vidioc_s_parm,
};

// receive a TYP01 screen
static void cgscreen_recv01(struct cgscreen_dev *dev, uint32_t off) {
	struct usb_interface *intf = dev->intf;
	uint8_t sum = 0;
	const uint8_t *p, *end;
	uint8_t *tmp;
	unsigned long flags;

	p = dev->urb_buf + ((off + 1) % URB_BUFSIZE);
	end = dev->urb_buf + ((off + URB_RX - 2) % URB_BUFSIZE);
	while (p != end) {
		sum += *p;
		if (++p == dev->urb_buf + URB_BUFSIZE)
			p = dev->urb_buf;
	}
	sum += hextoint(dev->urb_buf[(off + URB_RX - 2) % URB_BUFSIZE]) << 4 |
	       hextoint(dev->urb_buf[(off + URB_RX - 1) % URB_BUFSIZE]);

	if (sum) {
		dev_info(&intf->dev, "Invalid checksum: %hhu", sum);
		return;
	}

	spin_lock_irqsave(&dev->queue.lock, flags);
	tmp = dev->urb_buf;
	dev->urb_buf = dev->done_buf;
	dev->done_buf = tmp;
	dev->done_off = off;
	++dev->done_seq;
	dev->done_timestamp = ktime_get();
	spin_unlock_irqrestore(&dev->queue.lock, flags);
	// wake_up_all_locked(&dev->queue);
	dev->urb->transfer_buffer = dev->urb_buf;
}

static void cgscreen_read_bulk_callback(struct urb *urb) {
	struct cgscreen_dev *dev = urb->context;
	struct device *logdev = &dev->intf->dev;
	if (urb->status == -ESHUTDOWN || // device shutting down
	    urb->status == -EPROTO ||    // device shutting down
	    urb->status == -ECONNRESET)  // the urb was canceled
		return;
	if (urb->status)
		dev_info(logdev, "Read error %d", urb->status);
	else if (urb->actual_length == 0 ||         // ignore null reads
	         urb->actual_length == URB_BUFSIZE) // continue reading
		;
	else {
		uint32_t off = (urb->actual_length - URB_RX + URB_BUFSIZE) % URB_BUFSIZE;
		char type[6]; // packet type:1, screen type:5
		uint32_t i;
		for (i = 0; i < sizeof type; ++i)
			type[i] = dev->urb_buf[(off + i) % URB_BUFSIZE];
		if (type[0] != 0x0b)
			dev_info(logdev, "Got non-image packet type, ignoring");
		else if (memcmp(type + 1, "TYP01", sizeof type - 1) == 0)
			cgscreen_recv01(dev, off);
		else
			dev_info(logdev, "Got unknown screen type, ignoring");
	}
	usb_submit_urb(urb, GFP_KERNEL);
}

static int cgscreen_probe(struct usb_interface *intf, const struct usb_device_id *id) {
	struct cgscreen_dev *dev;
	struct usb_endpoint_descriptor *bulk_in;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	kref_init(&dev->kref);
	dev->udev = usb_get_dev(interface_to_usbdev(intf));
	dev->intf = usb_get_intf(intf);
	dev->urb_buf = dev->buffers[0];
	dev->done_buf = dev->buffers[1];
	init_waitqueue_head(&dev->queue);

	// set up the endpoint information
	ret = usb_find_common_endpoints(intf->cur_altsetting, &bulk_in, NULL, NULL, NULL);
	if (ret) {
		dev_err(&intf->dev, "Could not find bulk-in endpoint\n");
		goto error;
	}
	dev->bulk_in_endp = bulk_in->bEndpointAddress;
	dev->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->urb) {
		ret = -ENOMEM;
		goto error;
	}

	dev->vdev = video_device_alloc();
	if (!dev->vdev) {
		ret = -ENOMEM;
		goto error;
	}
	dev->vdev->v4l2_dev = &dev->v4l2_dev;
	if (dev->udev->product)
		strscpy(dev->vdev->name, dev->udev->product);
	else
		snprintf(dev->vdev->name, sizeof(dev->vdev->name), "Casio calculator capture (%04x:%04x)",
		         le16_to_cpu(dev->udev->descriptor.idVendor),
		         le16_to_cpu(dev->udev->descriptor.idProduct));
	dev->vdev->fops = &cgscreen_fops;
	dev->vdev->ioctl_ops = &cgscreen_ioctl_ops;
	dev->vdev->release = video_device_release;
	dev->vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	//dev->vdev->dev_debug |= V4L2_DEV_DEBUG_FOP | V4L2_DEV_DEBUG_IOCTL | V4L2_DEV_DEBUG_IOCTL_ARG;
	video_set_drvdata(dev->vdev, dev);

	// save our data pointer in this interface device
	usb_set_intfdata(intf, dev);

	// we can register the device now, as it is ready
	ret = v4l2_device_register(&intf->dev, &dev->v4l2_dev);
	if (ret) {
		dev_err(&intf->dev, "Unable to register v4l2 device.\n");
		usb_set_intfdata(intf, NULL);
		goto error;
	}

	ret = video_register_device(dev->vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(&intf->dev, "Unable to register video device.\n");
		goto unregister;
	}

	usb_fill_bulk_urb(dev->urb, dev->udev, usb_rcvbulkpipe(dev->udev, dev->bulk_in_endp),
	                  dev->urb_buf, URB_BUFSIZE, cgscreen_read_bulk_callback, dev);
	ret = usb_submit_urb(dev->urb, GFP_KERNEL);
	if (ret) {
		dev_err(&intf->dev, "Failed submitting read URB, error %d\n", ret);
		goto unregister;
	}

	dev_info(&intf->dev, "Casio graphing device now attached.");

	return 0;

unregister:
	video_unregister_device(dev->vdev);
	v4l2_device_unregister(&dev->v4l2_dev);
error:
	cgscreen_delete_dev(&dev->kref);
	return ret;
}

static void cgscreen_disconnect(struct usb_interface *intf) {
	struct cgscreen_dev *dev;

	dev = usb_get_intfdata(intf);
	usb_set_intfdata(intf, NULL);

	video_unregister_device(dev->vdev);
	v4l2_device_unregister(&dev->v4l2_dev);

	usb_kill_urb(dev->urb);

	// decrement our usage count
	cgscreen_put_dev(dev);

	dev_info(&intf->dev, "Casio graphing device disconnected");
}

static struct usb_driver cgscreen_driver = {
		.name = NAME,
		.probe = cgscreen_probe,
		.disconnect = cgscreen_disconnect,
		.id_table = cgscreen_id_table,
};

module_usb_driver(cgscreen_driver);
