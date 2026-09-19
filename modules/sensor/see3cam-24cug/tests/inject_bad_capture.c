/* Test-only LD_PRELOAD shim. Alters two successful capture results in userspace;
 * never changes camera controls, USB traffic, or the kernel driver's buffers.
 * Use only with the isolated camera test, never a flight session.
 * Build: cc -shared -fPIC -O2 inject_bad_capture.c -ldl -o /tmp/inject_bad_capture.so
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <linux/videodev2.h>
#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>

static unsigned long dequeued;
static unsigned long injected;

int ioctl(int fd, unsigned long request, ...) {
    static int (*original)(int, unsigned long, ...);
    va_list args;
    va_start(args, request);
    void *arg = va_arg(args, void *);
    va_end(args);
    if (!original) original = dlsym(RTLD_NEXT, "ioctl");
    int result = original(fd, request, arg);
    /* The legacy backend passes ioctl commands through a signed int; mirror
     * the kernel's 32-bit command comparison after possible sign extension. */
    if (result == 0 && (unsigned int)request == (unsigned int)VIDIOC_DQBUF) {
        struct v4l2_buffer *buffer = arg;
        if (buffer->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
            ++dequeued;
            if (dequeued == 120) {
                buffer->flags |= V4L2_BUF_FLAG_ERROR;
                ++injected;
                fprintf(stderr, "INJECT_CAPTURE pid=%d error_flag frame=%lu\n", getpid(), dequeued);
            } else if (dequeued == 240 && buffer->bytesused > 2) {
                buffer->bytesused -= 2;
                ++injected;
                fprintf(stderr, "INJECT_CAPTURE pid=%d partial_length frame=%lu\n", getpid(), dequeued);
            }
        }
    }
    return result;
}

__attribute__((destructor)) static void report(void) {
    if (dequeued) fprintf(stderr, "INJECT_CAPTURE_FINAL pid=%d dequeued=%lu injected=%lu\n", getpid(), dequeued, injected);
}
