#include <linux/container_of.h>
#include <linux/dev_printk.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/usb.h>
#include <linux/version.h>
#include <linux/videodev2.h>
#include <linux/vmalloc.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

MODULE_LICENSE("Dual MIT/GPL");
MODULE_AUTHOR("Baltazár Radics");
MODULE_DESCRIPTION("Casio graphing calculator screen projector USB driver");
MODULE_VERSION("0.1.0");

#define WIDTH 128
#define HEIGHT 64
#define SCREEN_SIZE (WIDTH * HEIGHT / 8UL)
#define URB_RX (6 + SCREEN_SIZE + 2)
#define URB_BLOCKSIZE 512
#define URB_BUFSIZE round_up(URB_RX, URB_BLOCKSIZE)
#define VIDEO_SIZE (SCREEN_SIZE * 8)
#define FOURCC V4L2_PIX_FMT_GREY

#define hextoint(hexchar) ((hexchar) <= '9' ? (hexchar) - '0' : (hexchar) + 10 - 'A')

#define to_casio4l_buf(buf) \
	container_of(buf, struct casio4l_buf, vb)

typedef uint8_t u8;

struct casio4l_buf {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

struct casio4l {
	struct kref kref;
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct usb_interface *intf;
	struct usb_device *udev;
	struct urb *urb;
	struct vb2_queue queue;
	struct list_head bufs; // struct casio4l_buf
	struct mutex lock;
	spinlock_t irqlock;
	unsigned seqnr;
	bool run:1;
	bool running:1;
	u8 urb_buf[URB_BUFSIZE];
};

static void casio4l_release(struct kref *kref) {
	struct casio4l *dev = container_of(kref, struct casio4l, kref);
	usb_free_urb(dev->urb);
	usb_put_dev(dev->udev);
	usb_put_intf(dev->intf);
	kfree(dev);
}

static void casio4l_put(struct casio4l *dev) {
	kref_put(&dev->kref, casio4l_release);
}

static const struct v4l2_file_operations casio4l_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
	.read = vb2_fop_read,
#ifndef CONFIG_MMU
	.get_unmapped_area = vb2_fop_get_unmapped_area,
#endif
};

static int casio4l_vidioc_querycap(struct file *file, void *priv, struct v4l2_capability *cap) {
	struct casio4l *dev = video_drvdata(file);
	if (!dev)
		return -ENODEV;

	strscpy(cap->driver, "casio4l");
	strscpy(cap->card, dev->vdev.name);
	usb_make_path(dev->udev, cap->bus_info, sizeof(cap->bus_info));
	cap->version = LINUX_VERSION_CODE;
	cap->capabilities = dev->vdev.device_caps | V4L2_CAP_DEVICE_CAPS;
	cap->device_caps = dev->vdev.device_caps;
	memset(cap->reserved, 0, sizeof(cap->reserved));

	return 0;
}

static int casio4l_vidioc_enum_input(struct file *file, void *fh, struct v4l2_input *inp) {
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

static int casio4l_vidioc_g_input(struct file *file, void *fh, unsigned int *i) {
	*i = 0;
	return 0;
}

static int casio4l_vidioc_s_input(struct file *file, void *fh, unsigned int i) {
	if (i != 0)
		return -EINVAL;
	return 0;
}

static int casio4l_vidioc_enum_fmt_vid_cap(struct file *file, void *fh, struct v4l2_fmtdesc *f) {
	if (f->index) // we have only one format
		return -EINVAL;

	strscpy(f->description, "8 bpp, Greyscale (GREY)");
	f->pixelformat = FOURCC;
	f->flags = 0;

	return 0;
}

static int casio4l_vidioc_g_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *fmt) {
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

static int casio4l_vidioc_g_parm(struct file *file, void *fh, struct v4l2_streamparm *a) {

	a->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	a->parm.capture.capturemode = 0;
	a->parm.capture.timeperframe.numerator = 1;
	a->parm.capture.timeperframe.denominator = 2;
	a->parm.capture.extendedmode = 0;
	a->parm.capture.readbuffers = 3;
	memset(a->parm.capture.reserved, 0, sizeof(a->parm.capture.reserved));

	return 0;
}

static const struct v4l2_ioctl_ops casio4l_ioctl_ops = {
	.vidioc_querycap = casio4l_vidioc_querycap,
	.vidioc_enum_input = casio4l_vidioc_enum_input,
	.vidioc_g_input = casio4l_vidioc_g_input,
	.vidioc_s_input = casio4l_vidioc_s_input,
	.vidioc_enum_fmt_vid_cap = casio4l_vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = casio4l_vidioc_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = casio4l_vidioc_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = casio4l_vidioc_g_fmt_vid_cap,
	.vidioc_g_parm = casio4l_vidioc_g_parm,
	// .vidioc_s_parm = casio4l_vidioc_g_parm,

	.vidioc_reqbufs     = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf    = vb2_ioctl_querybuf,
	.vidioc_qbuf        = vb2_ioctl_qbuf,
	.vidioc_dqbuf       = vb2_ioctl_dqbuf,
	.vidioc_streamon    = vb2_ioctl_streamon,
	.vidioc_streamoff   = vb2_ioctl_streamoff,
	.vidioc_expbuf      = vb2_ioctl_expbuf,
	.vidioc_remove_bufs = vb2_ioctl_remove_bufs,
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

// receive a TYP01 screen
static void casio4l_recv01(struct casio4l *dev, uint32_t off) {
	struct device *logdev = &dev->intf->dev;
	uint8_t sum = 0;
	const uint8_t *p, *end;
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
		dev_info(logdev, "Invalid checksum: %hhu", sum);
		return;
	}

	dev_info(logdev, "got valid typ01 frame");

	spin_lock_irqsave(&dev->irqlock, flags);
	if (list_empty(&dev->bufs)) {
		dev_info(logdev, "out of buffers!");
		spin_unlock_irqrestore(&dev->irqlock, flags);
		return;
	}
	struct casio4l_buf *buf = list_first_entry(&dev->bufs, struct casio4l_buf, list);
	list_del(&buf->list);
	spin_unlock_irqrestore(&dev->irqlock, flags);

	render01(dev->urb_buf, off, vb2_plane_vaddr(&buf->vb.vb2_buf, 0));
	vb2_set_plane_payload(&buf->vb.vb2_buf, 0, VIDEO_SIZE);
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.field = V4L2_FIELD_NONE;
	buf->vb.sequence = dev->seqnr++;
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
}

static void casio4l_urb_complete(struct urb *urb) {
	struct casio4l *dev = urb->context;
	struct device *logdev = &dev->intf->dev;
	if (urb->status == -ESHUTDOWN || // device shutting down
	    urb->status == -EPROTO ||    // device shutting down
	    urb->status == -ECONNRESET)  // the urb was canceled
		return;
	if (urb->status) {
		dev_info(logdev, "Read error %d", urb->status);
		goto resubmit;
	}
	dev_info(logdev, "got %i bytes", urb->actual_length);

	// print_hex_dump_bytes("casio4l ", DUMP_PREFIX_OFFSET, dev->urb_buf, urb->actual_length);

	if (urb->actual_length == 0 ||         // ignore null reads
			urb->actual_length == URB_BUFSIZE) // continue reading
		goto resubmit;

	uint32_t off = (urb->actual_length - URB_RX + URB_BUFSIZE) % URB_BUFSIZE;
	char hdr[6];
	uint32_t i;
	for (i = 0; i < sizeof hdr; ++i)
		hdr[i] = dev->urb_buf[(off + i) % URB_BUFSIZE];
	if (hdr[0] != 0x0b)
		dev_info(logdev, "Got non-image packet type, ignoring");
	else if (memcmp(hdr + 1, "TYP01", sizeof hdr - 1) == 0)
		casio4l_recv01(dev, off);
	else
		dev_info(logdev, "Got unknown screen type, ignoring");
resubmit:
	if (dev->run)
		usb_submit_urb(urb, GFP_KERNEL);
	else
		dev->running = false;
}

static int casio4l_queue_setup(struct vb2_queue *q,
		unsigned int *num_buffers, unsigned int *num_planes,
		unsigned int sizes[], struct device *alloc_devs[]) {
	if (*num_planes)
		return *num_planes != 1 || sizes[0] < VIDEO_SIZE ? -EINVAL : 0;

	*num_planes = 1;
	sizes[0] = VIDEO_SIZE;
	return 0;
}

static void casio4l_return_bufs(struct casio4l *dev, enum vb2_buffer_state state) {
	while (!list_empty(&dev->bufs)) {
		struct casio4l_buf *buf = list_first_entry(&dev->bufs, struct casio4l_buf, list);
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
}

static int casio4l_start_streaming(struct vb2_queue *q, unsigned int count) {
	struct casio4l *dev = container_of(q, struct casio4l, queue);
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&dev->irqlock, flags);

	if (!dev->running)
		ret = usb_submit_urb(dev->urb, GFP_KERNEL);
	if (ret == 0)
		dev->running = dev->run = true;
	else
		casio4l_return_bufs(dev, VB2_BUF_STATE_QUEUED);

	spin_unlock_irqrestore(&dev->irqlock, flags);

	return ret;
}

static void casio4l_stop_streaming(struct vb2_queue *q) {
	struct casio4l *dev = container_of(q, struct casio4l, queue);
	unsigned long flags;

	spin_lock_irqsave(&dev->irqlock, flags);
	dev->run = false;
	casio4l_return_bufs(dev, VB2_BUF_STATE_ERROR);
	spin_unlock_irqrestore(&dev->irqlock, flags);
}

static void casio4l_buf_queue(struct vb2_buffer *vb) {
	struct casio4l *dev = container_of(vb->vb2_queue, struct casio4l, queue);
	struct casio4l_buf *buf = to_casio4l_buf(to_vb2_v4l2_buffer(vb));
	unsigned long flags;

	spin_lock_irqsave(&dev->irqlock, flags);
	list_add_tail(&buf->list, &dev->bufs);
	spin_unlock_irqrestore(&dev->irqlock, flags);
}

struct vb2_ops casio4l_qops = {
	.queue_setup = casio4l_queue_setup,
	.start_streaming = casio4l_start_streaming,
	.stop_streaming = casio4l_stop_streaming,
	.buf_queue = casio4l_buf_queue,
};

static int casio4l_probe(struct usb_interface *intf, const struct usb_device_id *id) {
	int ret;

	struct usb_endpoint_descriptor *bulk_in;
	ret = usb_find_common_endpoints(intf->cur_altsetting, &bulk_in, NULL, NULL, NULL);
	if (ret)
		return ret;

	struct casio4l *dev = kzalloc(sizeof *dev, GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	kref_init(&dev->kref);
	dev->intf = usb_get_intf(intf);
	dev->udev = usb_get_dev(interface_to_usbdev(intf));
	mutex_init(&dev->lock);
	spin_lock_init(&dev->irqlock);
	INIT_LIST_HEAD(&dev->bufs);

	usb_set_intfdata(intf, dev);

	dev->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->urb) {
		ret = -ENOMEM;
		goto error;
	}

	struct vb2_queue *queue = &dev->queue;
	queue->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	queue->io_modes = VB2_MMAP | VB2_USERPTR | VB2_READ;
	queue->lock = &dev->lock;
	queue->ops = &casio4l_qops;
	queue->mem_ops = &vb2_vmalloc_memops;
	queue->buf_struct_size = sizeof(struct casio4l_buf);
	queue->min_queued_buffers = 2;
	queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	ret = vb2_queue_init(queue);
	if (ret)
		goto error;

	struct v4l2_device *v4l2_dev = &dev->v4l2_dev;
	ret = v4l2_device_register(&intf->dev, v4l2_dev);
	if (ret)
		goto error;

	struct video_device *vdev = &dev->vdev;
	vdev->v4l2_dev = v4l2_dev;
	strscpy(vdev->name, dev->udev->product);
	vdev->fops = &casio4l_fops;
	vdev->ioctl_ops = &casio4l_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE | V4L2_CAP_STREAMING;
	// vdev->dev_debug |= V4L2_DEV_DEBUG_FOP | V4L2_DEV_DEBUG_IOCTL | V4L2_DEV_DEBUG_IOCTL_ARG;
	vdev->queue = queue;
	video_set_drvdata(vdev, dev);
	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto error;

	usb_fill_bulk_urb(dev->urb, dev->udev, usb_rcvbulkpipe(dev->udev, bulk_in->bEndpointAddress), dev->urb_buf, URB_BUFSIZE, casio4l_urb_complete, dev);

	return 0;

	video_unregister_device(&dev->vdev);
	v4l2_device_unregister(&dev->v4l2_dev);
error:
	casio4l_release(&dev->kref);
	return ret;
}

static void casio4l_disconnect(struct usb_interface *intf) {
	struct casio4l *dev = usb_get_intfdata(intf);
	v4l2_device_disconnect(&dev->v4l2_dev);
	video_unregister_device(&dev->vdev);
	v4l2_device_unregister(&dev->v4l2_dev);
	usb_kill_urb(dev->urb);
	casio4l_put(dev);
}

static const struct usb_device_id casio4l_id_table[] = {
		{USB_DEVICE(0x07cf, 0x6101)}, // A caiso calculator in 'Projector' mode
		{}};
MODULE_DEVICE_TABLE(usb, casio4l_id_table);

static struct usb_driver casio4l_driver = {
		.name = KBUILD_MODNAME,
		.probe = casio4l_probe,
		.disconnect = casio4l_disconnect,
		.id_table = casio4l_id_table,
};

module_usb_driver(casio4l_driver);
