#include <linux/container_of.h>
#include <linux/dev_printk.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/timer_types.h>
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
#define URB_BUFSIZE 2048
#define VIDEO_SIZE (SCREEN_SIZE * 8)
#define FOURCC V4L2_PIX_FMT_GREY
#define SYNC_BYTE 0x0b

static_assert(URB_RX <= URB_BUFSIZE);

#define UMS_TAG "CA4L"

#define hextoint(hexchar) ((hexchar) <= '9' ? (hexchar) - '0' : (hexchar) + 10 - 'A')

#define to_casio4l_buf(buf) \
	container_of(buf, struct casio4l_buf, vb)

typedef uint8_t u8;

enum casio4l_prot {
	C4LP_BULK,
	C4LP_UMS,
};

enum casio4l_state {
	C4LS_STOPPED,
	// bulk mode:
	C4LS_B_RUN,
	// ums mode:
	C4LS_U_C1_CSW,
	C4LS_U_DELAY,
	C4LS_U_C0_CBW,
	C4LS_U_C0_RECV,
	C4LS_U_C0_CSW,
	C4LS_U_C1_CBW,
	C4LS_U_C1_RECV,
};

struct casio4l_buf {
	struct vb2_v4l2_buffer vb; // has to be first
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
	struct timer_list timer;
	spinlock_t irqlock;
	unsigned seqnr;
	unsigned rcvbulkpipe, sndbulkpipe;
	enum casio4l_prot prot;
	enum casio4l_state state;
	bool run:1;
	u8 urb_buf[URB_BUFSIZE];
	unsigned recvd;
	u8 recv_buf[URB_RX];
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

static int casio4l_vidioc_querycap(struct file *file, void *fh, struct v4l2_capability *cap) {
	struct casio4l *dev = video_drvdata(file);
	if (!dev)
		return -ENODEV;

	strscpy(cap->driver, KBUILD_MODNAME);
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

static int casio4l_vidioc_g_fmt_vid_cap(struct file *file, void *fh, struct v4l2_format *fmt) {

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

static void render01(uint8_t *dst, const uint8_t *src) {
	for (int i = 0; i < SCREEN_SIZE; ++i) {
		for (int b = 0x80; b; ++dst, b >>= 1)
			*dst = (src[i] & b) ? 0x00 : 0xff;
	}
}

// receive a TYP01 screen
static void casio4l_recv01(struct casio4l *dev) {
	struct device *logdev = &dev->intf->dev;
	uint8_t sum = 0;
	unsigned long flags;

	for (int i = 1; i < URB_RX - 2; ++i)
		sum += dev->recv_buf[i];
	sum += hextoint(dev->recv_buf[URB_RX - 2]) << 4 |
	       hextoint(dev->recv_buf[URB_RX - 1]);

	if (sum) {
		dev_info(logdev, "Invalid checksum: %hhu", sum);
		return;
	}

	// dev_info(logdev, "got valid typ01 frame");

	spin_lock_irqsave(&dev->irqlock, flags);
	if (list_empty(&dev->bufs)) {
		dev_info(logdev, "out of buffers!");
		spin_unlock_irqrestore(&dev->irqlock, flags);
		return;
	}
	struct casio4l_buf *buf = list_first_entry(&dev->bufs, struct casio4l_buf, list);
	list_del(&buf->list);
	spin_unlock_irqrestore(&dev->irqlock, flags);

	render01(vb2_plane_vaddr(&buf->vb.vb2_buf, 0), dev->recv_buf+6);
	vb2_set_plane_payload(&buf->vb.vb2_buf, 0, VIDEO_SIZE);
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.field = V4L2_FIELD_NONE;
	buf->vb.sequence = dev->seqnr++;
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
}

static void packet_done(struct casio4l *dev) {
	if (memcmp(dev->recv_buf + 1, "TYP01", 5) == 0)
		casio4l_recv01(dev);
}

static void casio4l_fill_ums_cbw(struct casio4l *dev, u32 len, u8 cmd) {
	dev->urb->pipe = dev->sndbulkpipe;
	dev->urb->transfer_buffer_length = 31;

	memset(dev->urb_buf, 0, 31);
	memcpy(dev->urb_buf, "USBC" UMS_TAG, 8);
	dev->urb_buf[8]  = (len >> 0)  & 0xFF;
	dev->urb_buf[9]  = (len >> 8)  & 0xFF;
	dev->urb_buf[10] = (len >> 16) & 0xFF;
	dev->urb_buf[11] = (len >> 24) & 0xFF;
	dev->urb_buf[12] = 0x80; // is input
	dev->urb_buf[14] = 16; // command size
	dev->urb_buf[15] = cmd;
	if (cmd == 0xc1) {
		dev->urb_buf[21] = (len >> 8) & 0xFF;
		dev->urb_buf[22] = (len >> 0) & 0xFF;
	}
}

static void casio4l_fill_ums_recv(struct casio4l *dev, u32 len) {
	dev->urb->pipe = dev->rcvbulkpipe;
	dev->urb->transfer_buffer_length = len;
}

static void casio4l_fill_ums_csw(struct casio4l *dev) {
	dev->urb->pipe = dev->rcvbulkpipe;
	dev->urb->transfer_buffer_length = 13;
}

// expects spinlock to be held
static int casio4l_cont(struct casio4l *dev) {
	if (!dev->run) switch (dev->state) {
	case C4LS_STOPPED:
	case C4LS_B_RUN:
	case C4LS_U_C0_CBW:
	case C4LS_U_C1_CBW:
		dev->state = C4LS_STOPPED;
		return 0;
	default:
		break;
	}

	if (dev->state == C4LS_STOPPED)
		dev->state = dev->prot == C4LP_BULK ? C4LS_B_RUN : C4LS_U_C0_CBW;

	switch (dev->state) {
	case C4LS_STOPPED:
		return 1;

	case C4LS_B_RUN:
		break;

	case C4LS_U_C0_CBW:
		casio4l_fill_ums_cbw(dev, 16, 0xC0);
		break;
	case C4LS_U_C0_RECV:
		casio4l_fill_ums_recv(dev, 16);
		break;
	case C4LS_U_C0_CSW:
		casio4l_fill_ums_csw(dev);
		break;
	case C4LS_U_C1_CBW:
		casio4l_fill_ums_cbw(dev, URB_RX, 0xC1);
		break;
	case C4LS_U_C1_RECV:
		casio4l_fill_ums_recv(dev, URB_RX);
		break;
	case C4LS_U_C1_CSW:
		casio4l_fill_ums_csw(dev);
		break;

	case C4LS_U_DELAY:
		dev->timer.expires = get_jiffies_64() + msecs_to_jiffies(10);
		add_timer(&dev->timer);
		return 0;
	}

	int ret = usb_submit_urb(dev->urb, GFP_KERNEL);
	if (ret)
		dev->state = C4LS_STOPPED;

	return ret;
}

static void casio4l_bulk_done(struct casio4l *dev) {
	if (dev->urb->actual_length == 0)
		packet_done(dev);

	for (unsigned i = 0; i < dev->urb->actual_length; ) {
		if (dev->recvd == 0 && dev->urb_buf[i] != SYNC_BYTE) {
			++i;
			continue;
		}
		unsigned rx = min(URB_BUFSIZE - i, URB_RX - dev->recvd);
		memcpy(dev->recv_buf + dev->recvd, dev->urb_buf + i, rx);
		dev->recvd += rx;
		i += rx;
		if (dev->recvd == URB_RX) {
			packet_done(dev);
			dev->recvd = 0;
		}
	}
}

static void casio4l_ums_done(struct casio4l *dev) {
	memcpy(dev->recv_buf, dev->urb_buf, URB_RX);
	packet_done(dev);
}

static void casio4l_chk_csw(struct casio4l *dev) {
	if (memcmp(dev->urb_buf, "USBS" UMS_TAG, 8)) {
		dev_err(&dev->intf->dev, "invalid csw");
		dev->run = false;
	}
}

static void casio4l_done(struct casio4l *dev) {
	int avail;
	switch (dev->state) {
	case C4LS_STOPPED:
		break;
	case C4LS_B_RUN:
		casio4l_bulk_done(dev);
		break;
	case C4LS_U_C0_CBW:
	case C4LS_U_C1_CBW:
	case C4LS_U_DELAY:
		++dev->state;
		break;
	case C4LS_U_C0_RECV:
		avail = (dev->urb_buf[6] << 8) | dev->urb_buf[7];
		if (avail < SCREEN_SIZE)
			dev->state = C4LS_U_C1_CSW;
		else
			dev->state = C4LS_U_C0_CSW;
		break;
	case C4LS_U_C0_CSW:
	case C4LS_U_C1_CSW:
		++dev->state;
		casio4l_chk_csw(dev);
		break;
	case C4LS_U_C1_RECV:
		casio4l_ums_done(dev);
		dev->state = C4LS_U_C1_CSW;
		break;
	};

	casio4l_cont(dev);
}

static void casio4l_timerfn(struct timer_list *timer) {
	struct casio4l *dev = container_of(timer, struct casio4l, timer);
	casio4l_done(dev);
}

static void casio4l_urb_complete(struct urb *urb) {
	struct casio4l *dev = urb->context;
	struct device *logdev = &dev->intf->dev;
	if (urb->status == -ESHUTDOWN || // device shutting down
	    urb->status == -EPROTO ||    // device shutting down
	    urb->status == -ECONNRESET)  // the urb was canceled
		return;
	if (urb->status) {
		dev_info(logdev, "Xfer error %d", urb->status);
		dev->state = C4LS_STOPPED; // TODO recover from errors?
		return;
	}

	// dev_info(logdev, "got %i bytes", urb->actual_length);
	// print_hex_dump_bytes("casio4l ", DUMP_PREFIX_OFFSET, dev->urb_buf, urb->actual_length);

	casio4l_done(dev);
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

	dev->seqnr = 0;
	dev->run = true;
	if (dev->state == C4LS_STOPPED)
		ret = casio4l_cont(dev);
	if (ret)
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

static const struct vb2_ops casio4l_qops = {
	.queue_setup = casio4l_queue_setup,
	.start_streaming = casio4l_start_streaming,
	.stop_streaming = casio4l_stop_streaming,
	.buf_queue = casio4l_buf_queue,
};

static int casio4l_probe(struct usb_interface *intf, const struct usb_device_id *id) {
	int ret;

	struct usb_endpoint_descriptor *bulk_in, *bulk_out;
	ret = usb_find_common_endpoints(intf->cur_altsetting, &bulk_in, &bulk_out, NULL, NULL);
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
	dev->rcvbulkpipe = usb_rcvbulkpipe(dev->udev, bulk_in->bEndpointAddress);
	dev->sndbulkpipe = usb_sndbulkpipe(dev->udev, bulk_out->bEndpointAddress);
	switch (id->idProduct) {
	case 0x6101:
		dev->prot = C4LP_BULK;
		break;
	case 0x6102:
		dev->prot = C4LP_UMS;
		break;
	}
	dev->timer.function = casio4l_timerfn;

	usb_set_intfdata(intf, dev);

	dev->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->urb) {
		ret = -ENOMEM;
		goto error;
	}
	usb_fill_bulk_urb(dev->urb, dev->udev, dev->rcvbulkpipe, dev->urb_buf, URB_BUFSIZE, casio4l_urb_complete, dev);

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
	vdev->queue = queue;
	video_set_drvdata(vdev, dev);
	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto error;

	dev_info(&intf->dev, "New calculator attached!");

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
		{USB_DEVICE_INTERFACE_CLASS(0x07cf, 0x6102, USB_CLASS_MASS_STORAGE)}, // A caiso calculator in 'ScreenRecv' mode
		{}};
MODULE_DEVICE_TABLE(usb, casio4l_id_table);

static struct usb_driver casio4l_driver = {
		.name = KBUILD_MODNAME,
		.probe = casio4l_probe,
		.disconnect = casio4l_disconnect,
		.id_table = casio4l_id_table,
};

module_usb_driver(casio4l_driver);
