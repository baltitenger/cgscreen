#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Baltazár Radics");
MODULE_DESCRIPTION("Casio graphing calculator screen projector usb driver");
MODULE_VERSION("0.1.0");

#define NAME "cgscreen"
#define SCREEN_SIZE (128*64 / 8ul)
#define URB_BUFSIZE (6 + SCREEN_SIZE + 2)

static const struct usb_device_id cgscreen_id_table[] = {
	{ USB_DEVICE(0x07cf, 0x6101) }, // A caiso calculator in 'Projector' mode
	{ }
};
MODULE_DEVICE_TABLE(usb, cgscreen_id_table);

static struct usb_driver cgscreen_driver;

struct cgscreen_dev {
	struct kref kref;
	struct usb_device *udev;         // the usb device for this device
	struct usb_interface *intf;      // the interface for this device
	struct urb *urb;                 // the urb to read data with
	uint8_t bulk_in_endp;            // the enpoint pipe for reading
	uint8_t *urb_buf;                // the buffer to receive data
	uint8_t *done_buf;               // a complete frame
	uint8_t buffers[2][URB_BUFSIZE]; // the buffers themselves
	struct mutex io_mutex;           // only one reader at a time
	uint8_t disconnected: 1;         // prevent io after disconnect
	struct v4l2_device v4l2_dev;     // video4linux device
	struct video_device *vdev;       // video device
};

static void cgscreen_delete(struct kref *kref) {
	struct cgscreen_dev *dev = container_of(kref, struct cgscreen_dev, kref);

	usb_free_urb(dev->urb);
	usb_put_intf(dev->intf);
	usb_put_dev(dev->udev);
	kfree(dev);
}

static int cgscreen_open(struct file *file) {
	struct cgscreen_dev *dev;

	dev = video_drvdata(file);
	if (!dev) {
		return -ENODEV;
	}

	// increment our usage count for the device
	kref_get(&dev->kref);

	return 0;
}

static ssize_t cgscreen_read(struct file *file, char *buffer, size_t count, loff_t *ppos) {
	struct cgscreen_dev *dev;
	int ret;

	return -EFAULT;

	if (!count)
		return 0;

	dev = video_drvdata(file);

	ret = mutex_lock_interruptible(&dev->io_mutex);
	if (ret < 0)
		return ret;

	if (dev->disconnected) {
		ret = -ENODEV;
		goto exit;
	}

	ret = min(count, SCREEN_SIZE);
	if (copy_to_user(buffer, dev->done_buf+6, ret)) {
		ret = -EFAULT;
	}

exit:
	mutex_unlock(&dev->io_mutex);
	return ret;
}

static int cgscreen_release(struct file *file) {
	struct cgscreen_dev *dev;

	dev = video_drvdata(file);
	if (!dev)
		return -ENODEV;

	// decrement the count on our device
	kref_put(&dev->kref, cgscreen_delete);

	return 0;
}

static const struct v4l2_file_operations cgscreen_fops = {
	.owner   = THIS_MODULE,
	.open    = cgscreen_open,
	.read    = cgscreen_read,
	.release = cgscreen_release,
};

void cgscreen_recv(struct cgscreen_dev *dev) {
	struct usb_interface *intf = dev->intf;
	uint8_t sum = 0;
	unsigned i;
	char *tmp;

	for (i = 1; i < URB_BUFSIZE - 2; ++i) {
		sum += dev->urb_buf[i];
	}
	sum += (dev->urb_buf[i] <= '9' ? dev->urb_buf[i] - '0' : dev->urb_buf[i] + 10 - 'A') << 4
		| (dev->urb_buf[i+1] <= '9' ? dev->urb_buf[i+1] - '0' : dev->urb_buf[i+1] + 10 - 'A');

	if (sum) {
		dev_info(&intf->dev, "Invalid checksum: %hhu", sum);
		return;
	}
	mutex_lock(&dev->io_mutex);
	tmp = dev->urb_buf;
	dev->urb_buf = dev->done_buf;
	dev->done_buf = tmp;
	mutex_unlock(&dev->io_mutex);
	dev->urb->transfer_buffer = dev->urb_buf;
}

void cgscreen_read_bulk_callback(struct urb *urb) {
	struct cgscreen_dev *dev = urb->context;
	struct usb_interface *intf = dev->intf;
	if (urb->status == -EOVERFLOW)
		dev_info(&intf->dev, "Got overflow, ignoring"); // seems to happen when screen updates a lot
	else if (urb->status == -EPROTO)
		return; // don't continue, device shutting down
	else if (urb->status)
		dev_info(&intf->dev, "Read error %d", urb->status);
	else if (urb->actual_length == 0) {} // ignore null reads
	else if (urb->actual_length != URB_BUFSIZE)
		dev_info(&intf->dev, "Got partial frame, ignoring");
	else if (dev->urb_buf[0] != 0x0b) 
		dev_info(&intf->dev, "Got non-image packet type, ignoring");
	else if (memcmp(dev->urb_buf+1, "TYP01", 5))
		dev_info(&intf->dev, "Git unknown screen type, ignoring");
	else
		cgscreen_recv(dev);
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
	dev->urb_buf  = dev->buffers[0];
	dev->done_buf = dev->buffers[1];

	// set up the endpoint information
	ret = usb_find_common_endpoints(intf->cur_altsetting,
			&bulk_in, NULL, NULL, NULL);
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
		strscpy(dev->vdev->name, dev->udev->product, sizeof(dev->vdev->name));
	else
		snprintf(dev->vdev->name, sizeof(dev->vdev->name),
			 "Casio calculator capture (%04x:%04x)",
			 le16_to_cpu(dev->udev->descriptor.idVendor),
			 le16_to_cpu(dev->udev->descriptor.idProduct));
	dev->vdev->fops         = &cgscreen_fops;
	// dev->vdev->ioctl_ops    = &cgscreen_ioctl_ops;
	dev->vdev->release      = video_device_release;
	dev->vdev->device_caps  = V4L2_CAP_DEVICE_CAPS | V4L2_CAP_VIDEO_CAPTURE;
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

	usb_fill_bulk_urb(dev->urb, dev->udev,
			usb_rcvbulkpipe(dev->udev, dev->bulk_in_endp),
			dev->urb_buf, URB_BUFSIZE,
			cgscreen_read_bulk_callback, dev);
	ret = usb_submit_urb(dev->urb, GFP_KERNEL);
	if (ret) {
		dev_err(&intf->dev, "Failed submitting read urb, error %d\n", ret);
		goto unregister;
	}

	dev_info(&intf->dev, "Casio graphing device now attached.");

	return 0;

unregister:
	video_unregister_device(dev->vdev);
	v4l2_device_unregister(&dev->v4l2_dev);
error:
	cgscreen_delete(&dev->kref);
	return ret;
}

static void cgscreen_disconnect(struct usb_interface *intf) {
	struct cgscreen_dev *dev;

	dev = usb_get_intfdata(intf);
	usb_set_intfdata(intf, NULL);

	video_unregister_device(dev->vdev);
	v4l2_device_unregister(&dev->v4l2_dev);

	// prevent more I/O from starting
	mutex_lock(&dev->io_mutex);
	dev->disconnected = 1;
	mutex_unlock(&dev->io_mutex);
	
	usb_kill_urb(dev->urb);

	// decrement our usage count
	kref_put(&dev->kref, cgscreen_delete);

	dev_info(&intf->dev, "Casio graphing device disconnected");
}

static struct usb_driver cgscreen_driver = {
	.name       = NAME,
	.probe      = cgscreen_probe,
	.disconnect = cgscreen_disconnect,
	.id_table = cgscreen_id_table,
};

module_usb_driver(cgscreen_driver);

