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
pointers to allow creating generic interfaces. This is similar to a *vtable* in
higher level languages like C++. For defining the behavior of our driver, we
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

[eiaf]: https://en.wikipedia.org/wiki/Everything_is_a_file

We could do the same, but then we'd still need to write an user-space program to
talk to this special device and presumably display the received screen contents.
However, Linux already has an API for streaming video data: V4L2 (Video4Linux).
Since it's a known and documented interface, many user-space applications exist
already to interact with V4L2 devices.

To do this, we need to create and register two different devices: `v4l2_device`
and `video_device`. The reason for this separation is that a single device could
contain more than one video device, or even other types of devices that belong
to the V4L2 framework (e.g. subdevices or radio tuners). The `v4l2_device`
instance serves as a parent for all devices belonging to the same physical
device. By registering a `video_device`, the V4L2 subsystem creates a device
node for us similar to the usb-skeleton example, but this time it gets a name
like `/dev/video0` (and also uses the `video4linux` subsystem's *major number*).

As for any other character device, we need to provide a `file_operations`
structure to specialize the behavior of various system calls on our module's
devices. V4L2 provides its own `struct v4l2_file_operations`, which is a subset
of the generic `struct file_operations` which only includes the calls needed for
V4L2 devices. It also has `video_ioctl2()`, which can be used for the
`unlocked_ioctl` field of fops, and uses a separate `struct v4l2_ioctl_ops` to
handle all the V4L2 specific `ioctl` calls. It also handles all the common logic
and validation of these `ioctl`s so drivers have less to worry about.

These `ioctl`s are the primary way for users-space to interact with the devices,
allowing apps to query and change many of their properties, including inputs,
outputs, pixel formats, frame intervals, controls. The V4L2 framework supports
many different modes of operation, so drivers typically only implement a subset
of these `ioctl`s (e.g. we only have video capture, so we won't have anything
output or audio related). The `VIDIOC_QUERYCAP` `ioctl` is used to communicate
what is and what isn't supported.

Once the parameters are negotiated successfully, apps can start streaming
frames. V4L2 defines two main ways of doing this: by using `read()` (or
`write()` in case of outputs, called `READWRITE`), or by exchanging
memory-mapped buffers (called `STREAMING`). The former is the simplest method,
but the latter can be more flexible in certain situations. The `STREAMING` API
allows exchanging meta-information about the buffers, like timestamps. In case
the driver supports DMA (Direct Memory Access), it may also be more efficient,
since it doesn't require an extra copy between kernel and user space.

While drivers technically only need to implement one of the methods, supporting
2more can allow a wider range of clients to work with our device. Normally this
would require quite a lot of boilerplate code: `READWRITE` needs `read()` and
`poll()`, `STREAMING` needs `mmap()`, `poll()`, and 5-6 extra `ioctl`s depending
on what method they support for buffer allocation. Since this would have to be
done in many drivers, the kernel developers came up with a common abstraction:
the `videobuf2` API.

Drivers using this API only need to implement a few callbacks in `struct
vb2_ops`, set up a `vb2_queue` object, and they can use the helpers provided by
`videobuf2` in place of the `file_operations` and `ioctl_ops` callbacks. This
mechanism also allows drivers to use a different lock for streaming `ioctl`s.

The `v4l-utils` project also provides a tool called `v4l2-compliance`, which can
check whether a driver behaves in accordance to the spec. It tests almost all
`ioctl`s and all streaming methods. This can be very useful when developing
a V4L2 driver, as it can test many more code paths than any single client.

## URB

Now that we have the basic structure of both the USB and V4L2 components set up,
we can finally get to the useful part of the driver: actually communicating with
the calculator.

The simplest protocol it knows uses USB bulk transfers. I won't go into too much
detail about the USB protocol here as it's fairly complicated, for our driver
we'll only have to understand that USB devices can have multiple endpoints with
different functions but we'll only have to use the bulk in and bulk out
endpoints. Once we have them, we can initiate transfers between the host and the
device using `URB`s (USB Request Blocks). The kernel's interface for this uses
the `urb` struct for keeping track of endpoints, buffers, and callbacks. This is
an asynchronous API, but Linux also has various synchronous wrappers for it.

The basic flow should look like this: when user-space wants to receive frames,
the `videobuf2` framework calls our `vb2_ops.start_streaming` callback. From
this point on, we should continuously receive frames from the calculator and
forward them to user-space until we are asked to stop via
`vb2_ops.stop_streaming`. This is fairly simple to do with the calculator's
interface, since it will only respond to the bulk in request once it has a frame
ready for us. An interesting quirk of this is that we only emit frames when the
screen changes.

## UMS

There are many other calculator models made by CASIO, quite a few of them
support some kind of screen capture. However, some of them use a different
protocol based on USB Mass Storage. It's a well established protocol that
exposes a SCSI interface over USB. Calculators in this mode show up as SCSI
devices with no medium present. For transferring frames, they use vendor
specific SCSI commands, so the builtin Linux driver can't do much with them on
its own. Thankfully, the model I have happens to support both protocols, so
I can easily test the driver.

Technically, I could set up a new driver for the SCSI subsystem, but I decided
to stick with USB. This keeps the driver's structure a bit simpler, and
I haven't found a simple API in Linux for sending SCSI custom commands from
drivers. Besides, the UMS protocol isn't too complex, especially the small
subset of it that this driver would need. It's still quite a bit more
complicated than the basic bulk transfer based protocol I described above.

The main command used for reading frames is `0xC1`: the host has to specify the
number of bytes it wants to read from the calculator's internal buffer. There's
one important difference from the simpler protocol though: this mode doesn't
support blocking, and instead if there aren't enough bytes available, the read
operation will report a failure (`EPIPE`) and will refuse to respond to further
queries until the fail condition is reset via `usb_clear_halt`.

This isn't the most ideal way to use USB, so the calculators implement
a different command, `0xC0` which can be used to query the number of bytes
available to receive. So that's what the driver does: when streaming is
requested, it starts polling (repeatedly querying) the calculator using command
`0xC0`, and once it reports that it has bytes available for reading, it reads
them using `0xC1`.

Of course, it's necessary to add a small delay between the queries, otherwise
this would essentially be a *busy loop* taking up both the host's and the
calculators processing time. (Compared to a modern computer's speed, the USB
transfer still takes a significant amount of time so this isn't such a big
issue, but running the queries without a delay does produce a visible slowdown
on the calculator's end.)

[kbuild]: https://docs.kernel.org/kbuild/modules.html#shared-makefile
[v4l2-core]: https://docs.kernel.org/driver-api/media/v4l2-core.html
[v4l2-user]: https://docs.kernel.org/userspace-api/media/v4l/user-func.html
