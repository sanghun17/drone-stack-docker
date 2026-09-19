#include <libuvc/libuvc.h>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <signal.h>
static std::atomic<unsigned long> count{0}, bad{0}, last_seq{0};
static volatile sig_atomic_t quit=0;
static void stop(int){quit=1;}
static void cb(uvc_frame_t* f,void*) {
 if(f->data_bytes != 1280*720*2) ++bad;
 last_seq=f->sequence; ++count;
}
int main(int argc,char**argv){
 setvbuf(stdout,nullptr,_IOLBF,0); signal(SIGINT,stop);signal(SIGTERM,stop);
 int seconds=argc>1?atoi(argv[1]):60, ret=1;
 uvc_context_t* ctx=nullptr;uvc_device_t* dev=nullptr;uvc_device_handle_t* h=nullptr;uvc_stream_ctrl_t ctrl{};
 auto check=[](uvc_error_t r,const char* name){printf("%s: %d\n",name,r);return r>=0;};
 if(!check(uvc_init(&ctx,nullptr),"init"))return 1;
 if(!check(uvc_find_device(ctx,&dev,0x2560,0xc128,"1A3958060A020900"),"find"))goto done;
 if(!check(uvc_open(dev,&h),"open"))goto done;
 uvc_print_diag(h,stderr);
 if(!check(uvc_get_stream_ctrl_format_size(h,&ctrl,UVC_FRAME_FORMAT_UYVY,1280,720,60),"negotiate"))goto done;
 uvc_print_stream_ctrl(&ctrl,stderr);
 check(uvc_set_ae_mode(h,1),"manual_exposure");check(uvc_set_exposure_abs(h,150),"exposure150");check(uvc_set_gain(h,1),"gain1");
 if(!check(uvc_start_streaming(h,&ctrl,cb,nullptr,0),"start"))goto done;
 { unsigned long prev=0;int stalled=0;
 for(int i=0;i<seconds&&!quit;++i){ std::this_thread::sleep_for(std::chrono::seconds(1));auto n=count.load();printf("second=%d total=%lu fps=%lu bad=%lu sequence=%lu\n",i+1,n,n-prev,bad.load(),last_seq.load());stalled=n==prev?stalled+1:0;prev=n;if(stalled>=8)break; }
 ret=count.load()>seconds*50&&bad.load()==0?0:2;
 }
 uvc_stop_streaming(h);
 done:if(h)uvc_close(h);if(dev)uvc_unref_device(dev);uvc_exit(ctx);printf("closed exit=%d\n",ret);return ret;
}
