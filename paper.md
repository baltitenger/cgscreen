# Linux driver for Casio graphing calculator screen sharing

## Intro

TODO: motivation, background

## USB Driver basics

The Linux source tree includes `drivers/usb/usb-skel.c`, which is meant as
a starting point for writing USB drivers, and demonstrates a few aspects of how
USB drivers and kernel modules work in general.

It's important to note that when writing kernel-mode code, we don't have access
to standard `libc` functions, only what's defined in the kernel headers. They do
include many interfaces that closely resemble their `libc` counterparts, for
example `<linux/string.h>` defines macros like `memcpy()` and `strscpy()`. It
also has many generic utilities, including a system for creating intrusive
linked lists, reference counting, dynamic memory management, and various
synchronization primitives.

In Linux, every module has a special entry and exit point function, which can be
marked with the `module_init(f)` and `module_exit(f)` macros respectively. Since
we're building an external module, these will be called whenever our module is
inserted or removed. However, since most modules primarily interact with
a single subsystem, there are convenience macros that set up a module for
a specific purpose. In our case, it's `module_usb_driver(usb_driver)` which
creates the init and exit functions for us for registering and deregistering our
USB driver automatically.

But what's an USB driver? The Linux kernel uses structs containing function
pointers to allow creating generic interfaces. This is similar to a *vtable*
in higher level languages like C++. For defining the behavior of our driver, we
need to fill out a `struct usb_driver`, allowing us to specify actions to take
when a device is connected or disconnected (`probe` and `disconnect`), power
management functions, and others. Here we can also include an ID table, which is
used by the kernel (or user-space tools) to determine what devices our driver's
supposed to handle. Various properties can be matched against, but in our case
it's enough to check the device's vendor and product ID.

So far, we have something like this:

```c
static int casio4l_probe(struct usb_interface *intf, const struct usb_device_id *id) {
  dev_info(&intf->dev, "New calculator attached!");
  return 0;
}

static void casio4l_disconnect(struct usb_interface *intf) {
  dev_info(&intf->dev, "Calculator detached.");
}

static const struct usb_device_id casio4l_id_table[] = {
  {USB_DEVICE(0x07cf, 0x6101)}, // A caiso calculator in 'Projector' mode
  {}};
MODULE_DEVICE_TABLE(usb, casio4l_id_table);

static struct usb_driver casio4l_driver = {
  .name       = KBUILD_MODNAME,
  .probe      = casio4l_probe,
  .disconnect = casio4l_disconnect,
  .id_table   = casio4l_id_table,
};

module_usb_driver(casio4l_driver);
```

This is almost enough to test it out, we just need a way to compile it. Linux
uses its own `kbuild` build system largely based on Makefiles that allows
configuring many aspects of the kernel through `kconfig`. After setting up
a Makefile and building our module, we get a `.ko` file we can load in the
kernel. Since it's not installed yet, we have to insert it manually using
`insmod casio4l.ko`. Sure enough, we can see the driver getting registered in
`dmesg`:

    usbcore: registered new interface driver casio4l

Once we actually connect a calculator (in the right mode), we can also see that
our `probe` function is called as we expect:

    usb 1-3: new high-speed USB device number 14 using xhci_hcd
    usb 1-3: New USB device found, idVendor=07cf, idProduct=6101, bcdDevice= 1.00
    usb 1-3: New USB device strings: Mfr=1, Product=2, SerialNumber=0
    usb 1-3: Product: CESG502
    usb 1-3: Manufacturer: CESG502
    casio4l 1-3:1.0: New calculator attached!

And on disconnect:

    usb 1-3: USB disconnect, device number 14
    casio4l 1-3:1.0: Calculator detached.

## Video4Linux

In its probe function, the usb-skeleton example sets up and registers an USB
class device. This would create a *character special* device entry in the `/dev`
special file system like `/dev/usb/skel0` whenever a device handled by the
driver is connected. This special file allows user-space code to communicate
with the USB device through the same API they use with regular files: `open`,
`read`, `write`, etc. (This is a [common design][eiaf] in Unix-like operating
systems.) The behavior of these system calls is defined by the module's `struct
file_operations`.

We could do the same, but then we'd still need to write an user-space program to
talk to this special device and presumably display the received screen contents.
However, Linux already has an API for streaming video data: `v4l2`
(Video4Linux). Since it's a known and documented interface, many user-space
applications exist already to interact with `v4l2` devices.

To do this, we need to create and register two different devices: `v4l2_device`
and `video_device`. The reason for this separation is that a single device could
contain more than one video device, or even other types of devices that belong
to the `v4l2` framework (e.g. subdevices or radio tuners). The `v4l2_device`
instance serves as a parent for all devices belonging to the same physical
device. By registering a `video_device`, the `v4l2` subsystem creates a device
node for us similar to the usb-skeleton example, but this time it gets a name
like `/dev/video0` (and also uses the `video4linux` subsystem's *major number*).

As for any other character device, we need to provide a `file_operations`
structure to specialize the behavior of various system calls on our module's
devices.

[kbuild]: https://docs.kernel.org/kbuild/modules.html#shared-makefile
[v4l2-core]: https://docs.kernel.org/driver-api/media/v4l2-core.html
[v4l2-user]: https://docs.kernel.org/userspace-api/media/v4l/user-func.html
[eiaf]: https://en.wikipedia.org/wiki/Everything_is_a_file
