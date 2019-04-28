// gcc -o GUIS -g  GNSS_USB_IF_Sampling.c -lusb-1.0 -lpthread -L/work/libusb/lib/ -I/work/libusb/include/libusb-1.0/
// 
// gcc -O2 -o GUIS -g  GNSS_USB_IF_Sampling.c -lusb-1.0 -lpthread -L/work/libusb/lib/ -I/work/libusb/include/libusb-1.0/

/*
1.环境
	ubuntu18.04
2.获取源代码
	下载 libusb-1.0.22.tar.bz2（https://sourceforge.net/projects/libusb/files/）
3.解压源码	
	$ cd /Downloads
	$ tar xzvf libusb-1.0.22.tar.bz2
4.按照INSTALL文件给出的提示进行安装：./configure - make - make install
	$ cd libusb-1.0.21/
	$ ./configure

configure失败，configure: error: "udev support requested but libudev not installed"

5.安装依赖项libudev-dev：	
	$ cd
	$ sudo apt-get install libudev-dev

6.安装成功后重新configure，成功，再make，make install，安装成功。
	$ make
	$ sudo make install　
	
7.编译文件并执行文件
    gcc GNSS_USB_IF_Sampling.c -o guis -I/usr/local/include/libusb-1.0  -L/usr/local/lib -lusb-1.0 
   （-I包含头文件 ;-L链接库）

    sudo ./guis // 執行文件，必须加上sudo

*/

#include <stdio.h>
#include <unistd.h>
#include <libusb.h>
//#include <pthread.h>
#include <string.h>
#include <fcntl.h>
//#include <stdlib.h>
//#include <fcntl.h>
//#include <time.h>

#define UBLOX
//#define IF_DATA_Collection

#ifdef IF_DATA_Collection
#define  USB_VID      0x2222
#define  USB_PID      0x5555
#endif

#ifdef UBLOX
#define USB_VID       0x10c4
#define USB_PID       0xea60
#endif

#define  EP_IN        0x0a//0x82
#define  BUF_SIZE     (16) // 64k buffer for one bulk 0x10000
#define  MAX_COUNT    3      // control total number of data, you should configure it before run this code#

#define  BUF_LENG      16
#define  MASK1         0x3FF     // 1023

#define EP_INTR			(1 | LIBUSB_ENDPOINT_IN)
#define EP_DATA			(2 | LIBUSB_ENDPOINT_IN)
#define CTRL_IN			(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)
#define CTRL_OUT		(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT)
#define USB_RQ			0x04
#define INTR_LENGTH		64


//static char buf[BUFSIZ];

#define  FOPEN_FUCNTION

int fd;
FILE *fp; // file pointer to file that write USB data
static struct libusb_device_handle *dev_handle  =  NULL; // USB handle
pthread_t pthid;
    
typedef enum
{
    false  = 0,
    true   = !false
} bool;
bool thread_exit = false; // close thread flag

struct BUFFER
{
    int n; // size buffer used
    char c[BUF_SIZE];
};

struct ChMeas // channel data measurement
{
    struct BUFFER buf[BUF_LENG];
    short inp;          // data in pointer
    short outp;       // data out pointer
};
struct ChMeas CHMEAS;

  //max2769 spi config
char spi1[10][64]={
    {0xA2,0x91,0x9A,0x30},//CONF1//{0xA2,0x91,0x9A,0x30},//CONF1
    {0x05,0x50,0x28,0xC1},//CONF2
    {0xEA,0xFE,0x1D,0xC2},//CONF3
    {0x9E,0xC0,0x00,0x83},//PLLCONF,{0x9E,0xC0,0x00,0x83},//PLLCONF
    {0x0C,0x00,0x08,0x04},//DIV,GPS:{0x0C,0x00,0x08,0x04},
    {0x00,0x00,0x07,0x05},//FDIV 
    {0x80,0x00,0x00,0x06},//STRM
    {0x10,0x06,0x1B,0x27},//CLK
    {0x1E,0x0F,0x40,0x18},//TEST1
    {0x14,0xC0,0x40,0x29}};//TEST2

  //max2769 spi config
char spi2[10][64]={
    {0xA2,0x91,0x9A,0x30},//CONF1
    {0x05,0x50,0x28,0xC1},//CONF2
    {0xEA,0xFE,0x1D,0xC2},//CONF3
    {0x9E,0xC0,0x00,0x83},//PLLCONF
    {0x0B,0xE4,0x08,0x04},//DIV,BD1:{0x0B,0xE4,0x08,0x04}
    {0x00,0x00,0x07,0x05},//FDIV,L1oíBD1:{0x00,0x00,0x07,0x05}
    {0x80,0x00,0x00,0x06},//STRM
    {0x10,0x06,0x1B,0x27},//CLK
    {0x1E,0x0F,0x40,0x18},//TEST1
    {0x14,0xC0,0x40,0x29}};//TEST2
    
void MAX2769_configure(void);
void ch_measure_init(void);
void chMeasure_push(void);
void chMeasure_push(void);
void chMeasure_pop(void);
struct BUFFER *chMeasure_front(void) ;
struct BUFFER *chMeasure_back(void);
short chMeasure_size(void);
void read_usb_data(void) ;
void write_usb_data(void);
// these three functions below unused
//void *read_usb_data_thread(void * arg);
//int open_thread(void) ;
//void close_thread(void) ;
static void print_devs(libusb_device **devs);
static int device_satus(libusb_device_handle *devh);
void display_usb_data(void);

int main(void) 
{
    int ret, usb_num;
    int  write_size, write_count = 0;
    libusb_device **devs;
    size_t cnt;
    
    //usleep(5000000); //a delay comment it when debugging
    
    // 初始化相关数据，必须在开始调用
    ret = libusb_init(NULL); 
    if (ret < 0)
    {
        printf("libusb_init error\n");
        return ret;
    }
    else
    {
        printf("Initlization succeed! ID:%d\r\n", ret);
    }
    
    cnt = libusb_get_device_list(NULL, &devs); 
    if (cnt < 0) 
       return (int) cnt;
    
    dev_handle = libusb_open_device_with_vid_pid(NULL, USB_VID, USB_PID);
    if(dev_handle == NULL)
    {
        perror("Cannot open device!\n");
    }
    else
    {   
        printf("Device Opened!\n");
    }

    print_devs(devs); 
    libusb_free_device_list(devs, 1);

    /*
    devh = libusb_open_device_with_vid_pid(NULL, MY_VID, MY_PID);

    if (devh == NULL)
    {
        printf("libusb_open_device_with_vid_pid error\n");
        return 1;
    }
    */

    //libusb_claim_interface(devh, 0);
    //libusb_set_configuration(devh,1);
    
    // confige chip 2769's GPS Channel
    //MAX2769_configure();
    
    ch_measure_init();
    
    
    fp = fopen("/home/grq-t470p/Documents/Projects/Cprogram/usbdata.txt", "w");
    if (fp == NULL)
    {
        printf(">>Open usbdata.dat file error, please check the file path!\n");
        return 1;
    }


    
    while(1)
    {
	
        read_usb_data();
      
        write_size = chMeasure_size();
        if (write_size > 0)
        {
            //write_usb_data();
            display_usb_data();
            write_count++;
        }
        
        // check the total number frame of data with the maximun count
        if (write_count < MAX_COUNT)
        {
            printf("%d, %d\n",write_count, write_size);//
        }
        else
        {
            break;
        }
  	
 /* 	
	    device_satus(dev_handle);
	    write_count++;
	    if (write_count > MAX_COUNT)
	    {
	        break;
	    }
*/
    }
    printf("write count: %d\n",write_count);
    

    fclose(fp); 


    libusb_exit(NULL); // close USB connnection

    return 0;
}


// 打印USB設備列表
static void print_devs(libusb_device **devs) 
{
    libusb_device *dev;
    int i = 0;
    
    while ((dev = devs[i++]) != NULL) 
    { 
        struct libusb_device_descriptor desc;
        int r = libusb_get_device_descriptor(dev, &desc); 
    
        if (r < 0) 
        {
            fprintf(stderr, "failed to get device descriptor"); 
            return; 
        } 
        printf("%04x:%04x (bus %d, device %d)\n", 
                desc.idVendor, 
                desc.idProduct, 
                libusb_get_bus_number(dev),
                libusb_get_device_address(dev)); 
    } 
}

static int device_satus(libusb_device_handle *devh)
{
 
	int interface = 0;
	unsigned char data[16];
	int r;
	unsigned int i;
	/*
	int LIBUSB_CALL libusb_control_transfer(libusb_device_handle *dev_handle,
	                                        uint8_t request_type, uint8_t bRequest, 
	                                        uint16_t wValue, 
	                                        uint16_t wIndex,
	                                        unsigned char *data, uint16_t wLength, unsigned int timeout);
	*/
	/*
	libusb_control_transfer(hd, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
			LIBUSB_REQUEST_CLEAR_FEATURE,
			0,
			interface,
			byte, 8, 1000);

    for (int i = 0; i < 16; ++i)
    {
        printf("status:0x%x\n", byte[i]);
    }
    */
    
    r = libusb_control_transfer(devh, CTRL_IN, USB_RQ, 0x00, 0, data, sizeof(data), 0);
	if (r < 0) {
		fprintf(stderr, "read hwstat error %d\n", r);
		return r;
	}
	if ((unsigned int) r < 1) {
		fprintf(stderr, "short read (%d)\n", r);
		return -1;
	}
	
	printf("F0 data:");
	for (i = 0; i < sizeof(data); i++)
		printf("%02x ", data[i]);
	printf("\n");
	
	return 0;
}


// RF chip max2769 configure
void MAX2769_configure(void)
{
    int i;
    for(i = 0; i < 10; i++)
    {
        libusb_control_transfer(dev_handle, 0x40, 0xeb, 0, 0, spi1[i], 4, 200);
        usleep(50000); 
    }
    
    // config chip 2769's BD channel
    for(i = 0; i < 10; i++)
    {
        libusb_control_transfer(dev_handle, 0x40, 0xec, 0, 0, spi2[i], 4, 200);
        usleep(50000); 
    }
}

void ch_measure_init(void)
{
    CHMEAS.inp    = 0;
    CHMEAS.outp   = 0;
}

void chMeasure_push(void)
{
    CHMEAS.inp++;
    CHMEAS.inp &= MASK1; // limit in range of 0 to 1023 due to BUF_LENG
}

void chMeasure_pop(void)
{
    CHMEAS.outp++;
    CHMEAS.outp &= MASK1;// limit in range of 0 to 1023 due to BUF_LENG
}

// channel data buffer front address
struct BUFFER *chMeasure_front(void) 
{
    return &CHMEAS.buf[CHMEAS.inp];
}

// channel data buffer back address
struct BUFFER *chMeasure_back(void)
{
    return &CHMEAS.buf[CHMEAS.outp];
}

short chMeasure_size(void)
{
    short length = CHMEAS.inp + BUF_LENG - CHMEAS.outp;
    length &= MASK1;
    return length;
}

void read_usb_data(void) // get data from USB
{
    int read_size;
    struct BUFFER *pbuf;
    
    pbuf = chMeasure_front(); // get data buffer front address
    
    /*libusb_bulk_transfer(struct libusb_device_handle *dev_handle,
     *                     unsigned char endpoint, 
     *                     unsigned char *data, 
     *                     int length, 
     *                     int *transferred,
     *                     unsigned int timeout) 
    */
    libusb_bulk_transfer(dev_handle, EP_IN, pbuf->c, BUF_SIZE, &read_size, 1000); 
    
    pbuf->n = read_size;
    // printf("read size: %d\n", read_size); // debug only 
    chMeasure_push();
}

void write_usb_data(void)
{
    struct BUFFER *pbuf;
        
    pbuf = chMeasure_back();

    fwrite(pbuf->c, sizeof(char), pbuf->n, fp);


    chMeasure_pop();
}

void display_usb_data(void)
{
	struct BUFFER *pbuf;
        
    pbuf = chMeasure_back();
    
    printf("USB data is:");

    for (int i = 0; i < pbuf->n; ++i)
    {
    	printf("%02x ", pbuf->c[i]);
    }		
	printf("\n");

    chMeasure_pop();
}

/*
void *read_usb_data_thread(void * arg)
{
    //if (!thread_exit)
    //{
        read_usb_data();
        // printf("read usb data\n"); // debug only
   // }
    return NULL;
}


int open_thread(void) // open a thread to get usb data
{
    //pthread_t pthid;
    
    int res = pthread_create(&pthid, NULL, read_usb_data_thread, NULL);
    if (res != 0)
    {
        printf("%s\n", strerror(res));
        return 1;
    }
    else
    {
        //printf("read_usb_data_thread id is: %d\n", pthid);
        return 0;
    }
}

void close_thread(void) // close thread 
{
    pthread_exit(NULL);
}
*/


