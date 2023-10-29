#include <stdio.h>
#include<ad9361.h>
#include<iio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#define KHZ(x) ((long long)(x*1000.0 + .5))
#define MHZ(x) ((long long)(x*1000000.0 + .5))
#define GHZ(x) ((long long)(x*1000000000.0 + .5))
#include<sys/select.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<netinet/in.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdlib.h>
#include<malloc.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <math.h>
#include <unistd.h>
#include <values.h>
#include <complex.h>
#include <fftw3.h>
#define IF_NAME "eth0"
#define ASSERT(expr) { \
	if (!(expr)) { \
		(void) fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
		(void) abort(); \
	} \
}

// powerctrl
#define GPIOpowerctrl_baseddr                         0x41200000

// radar lier parm ctrl
#define GPIOradarctrl_baseddr                         0x7C460000
static volatile uint32_t *gpio;
static volatile uint32_t *radarparmctrl;
#define MAXDATASIZE 65536
#define port_in 24576
#define port_out 24575
int currentswitchstate = 0;
#define IIO_ENSURE(expr) { \
	if (!(expr)) { \
		(void) fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
		(void) abort(); \
	} \
}

struct stream_cfg {
    long long bw_hz; // Analog banwidth in Hz
    long long fs_hz; // Baseband sample rate in Hz
    long long lo_hz; // Local oscillator frequency in Hz
    const char* rfport; // Port name
    double gain_db; // Hardware gain
	bool ispoweroff;
};
enum iodev { RX, TX };
struct stream_cfg txcfg;
struct stream_cfg rxcfg;
/* static scratch mem for strings */
static char tmpstr[64];
struct iio_context *ctx = NULL;
struct iio_device *tx = NULL;
struct iio_device *rx = NULL;
struct iio_device *phydev = NULL;
struct iio_channel *tx0_i = NULL;
struct iio_channel *tx0_q = NULL;
struct iio_channel *rx0_i = NULL;
struct iio_channel *rx0_q = NULL;
struct iio_buffer *tx_buffer = NULL;
struct iio_buffer *rx_buffer = NULL;
struct iio_scan_context *scan_ctx;
struct iio_context_info **info;

// short *prx_buffer;
// short *ptx_buffer;
bool repeat_stop = false;
FILE *fp = NULL;

bool stop;
const char *uri = NULL;
const char *ip = NULL;
// static double win_hanning(int j, int n)
// {
// 	double a = 2.0 * M_PI / (n - 1), w;
// 	w = 0.5 * (1.0 - cos(a * j));
// 	return (w);
// }
int get_local_ip(char * ifname, char * ip)
{
    char *temp = NULL;
    int inet_sock;
    struct ifreq ifr;

    inet_sock = socket(AF_INET, SOCK_DGRAM, 0); 

    memset(ifr.ifr_name, 0, sizeof(ifr.ifr_name));
    memcpy(ifr.ifr_name, ifname, strlen(ifname));

    if(0 != ioctl(inet_sock, SIOCGIFADDR, &ifr)) 
    {   
        perror("ioctl error");
        return -1;
    }

    temp = inet_ntoa(((struct sockaddr_in*)&(ifr.ifr_addr))->sin_addr);     
    memcpy(ip, temp, strlen(temp));

    close(inet_sock);

    return 0;
}

  

void * repeat_transfer(void * arg);
/* cleanup and exit */
static void shuTdown()
{
	printf("* Destroying buffers\n");
	
	if (tx_buffer) { iio_buffer_destroy(tx_buffer); }

	printf("* Disabling streaming channels\n");
	if (tx0_i) { iio_channel_disable(tx0_i); }
	if (tx0_q) { iio_channel_disable(tx0_q); }
    if (rx0_i) { iio_channel_disable(rx0_i); }
	if (rx0_q) { iio_channel_disable(rx0_q); }
	printf("* Destroying context\n");
	if (ctx) { iio_context_destroy(ctx); }
	exit(0);
}
/* check return value of attr_write function */
static void errchk(int v, const char* what) {
	 if (v < 0) { fprintf(stderr, "Error %d writing to channel \"%s\"\nvalue may not be supported.\n", v, what); shuTdown(); }
}

/* write attribute: long long int */
static void wr_ch_lli(struct iio_channel *chn, const char* what, long long val)
{
	errchk(iio_channel_attr_write_longlong(chn, what, val), what);
}

/* write attribute: string */
static void wr_ch_str(struct iio_channel *chn, const char* what, const char* str)
{
	errchk(iio_channel_attr_write(chn, what, str), what);
}

/* helper function generating channel names */
static char* get_ch_name(const char* type, int id)
{
	snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
	return tmpstr;
}

/* returns ad9361 phy device */
static struct iio_device* get_ad9361_phy(struct iio_context *ctx)
{
	struct iio_device *dev =  iio_context_find_device(ctx, "ad9361-phy");
	ASSERT(dev && "No ad9361-phy found");
	return dev;
}

/* finds AD9361 streaming IIO devices */
static bool get_ad9361_stream_dev(struct iio_context *ctx, enum iodev d, struct iio_device **dev)
{
	switch (d) {
	case TX: *dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc"); return *dev != NULL;
	case RX: *dev = iio_context_find_device(ctx, "cf-ad9361-lpc");  return *dev != NULL;
	default: ASSERT(0); return false;
	}
}

/* finds AD9361 streaming IIO channels */
static bool get_ad9361_stream_ch(struct iio_context *ctx, enum iodev d, struct iio_device *dev, int chid, struct iio_channel **chn)
{
	*chn = iio_device_find_channel(dev, get_ch_name("voltage", chid), d == TX);
	if (!*chn)
		*chn = iio_device_find_channel(dev, get_ch_name("altvoltage", chid), d == TX);
	return *chn != NULL;
}

/* finds AD9361 phy IIO configuration channel with id chid */
static bool get_phy_chan(struct iio_context *ctx, enum iodev d, int chid, struct iio_channel **chn)
{
	switch (d) {
	case RX: *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("voltage", chid), false); return *chn != NULL;
	case TX: *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("voltage", chid), true);  return *chn != NULL;
	default: ASSERT(0); return false;
	}
}

/* finds AD9361 local oscillator IIO configuration channels */
static bool get_lo_chan(struct iio_context *ctx, enum iodev d, struct iio_channel **chn)
{
	switch (d) {
	 // LO chan is always output, i.e. true
	case RX: *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("altvoltage", 0), true); return *chn != NULL;
	case TX: *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("altvoltage", 1), true); return *chn != NULL;
	default: ASSERT(0); return false;
	}
}

/* applies streaming configuration through IIO */
bool cfg_ad9361_streaming_ch(struct iio_context *ctx, struct stream_cfg *cfg, enum iodev type, int chid)
{
	struct iio_channel *chn = NULL;

	// Configure phy and lo channels
	printf("* Acquiring AD9361 phy channel %d\n", chid);
	if (!get_phy_chan(ctx, type, chid, &chn)) {	return false; }
	wr_ch_str(chn, "rf_port_select",     cfg->rfport);
	wr_ch_lli(chn, "rf_bandwidth",       cfg->bw_hz);
	wr_ch_lli(chn, "sampling_frequency", cfg->fs_hz);

	// Configure LO channel
	printf("* Acquiring AD9361 %s lo channel\n", type == TX ? "TX" : "RX");
	if (!get_lo_chan(ctx, type, &chn)) { return false; }
	wr_ch_lli(chn, "frequency", cfg->lo_hz);
	return true;
}

/* applies streaming configuration through IIO */

 int current_tx_length = 100000;
 int txlength = 4096*4;
int main(int argc, char *argv[])
{
	printf("start init resource.\r\n");
//pthread ctrl fan
   int fd;
printf("start gpio mmap\r\n");
   if ((fd = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC)) < 0) {
       printf("Unable to open /dev/mem: %s\n", strerror(errno));
       return -1;
   }
printf("success gpio mmap\r\n");
   gpio = (uint32_t *) mmap(0, getpagesize() * 250, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIOpowerctrl_baseddr);
   if ((void *) gpio == MAP_FAILED) {
       printf("mmap failed: %s\n", strerror(errno));
       exit(1);
   }

   int radarfd;
printf("start radarparmctrl mmap\r\n");
   if ((radarfd = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC)) < 0) {
       printf("Unable to open /dev/mem: %s\n", strerror(errno));
       return -1;
   }
printf("success radarparmctrl mmap\r\n");
   radarparmctrl = (uint32_t *) mmap(0, getpagesize() * 250, PROT_READ | PROT_WRITE, MAP_SHARED, radarfd, GPIOradarctrl_baseddr);
   if ((void *) radarparmctrl == MAP_FAILED) {
       printf("mmap failed: %s\n", strerror(errno));
       exit(1);
   }
 int wavelength,oldwavelength;
oldwavelength = 61440000/1000000*10;
 *(radarparmctrl) =oldwavelength;
 int capturetimes,oldcapturetimes;
oldcapturetimes = 10000;
*(radarparmctrl+1) =oldcapturetimes;
 int Countdowntime,oldCountdowntime;
oldCountdowntime = 3*61440000;
*(radarparmctrl+2) = oldCountdowntime;
 int maxdelaytime,oldmaxdelaytime;
oldmaxdelaytime = 61440000/1000*10;
*(radarparmctrl+3) =oldmaxdelaytime;
 int isContinuouslaunch,oldisContinuouslaunch;
oldisContinuouslaunch = 0;
*(radarparmctrl+4) = oldisContinuouslaunch;
 unsigned int bandwidth_khz_st = 40*1000;
  unsigned int bandwidth_khz = bandwidth_khz_st*14/10;
  
    printf("init gpio success...\n");
  
char linebuf[100] = {0}; // 行数据缓存
char *currentmod  =" bpsk";
  int line_num = 0;
  /**** ofdm data
****/

  short *ptx_buffer = (short*)malloc(txlength*2*2);
    memset(ptx_buffer,0,txlength*2*2);  
     short *prx_buffer = (short*)malloc(txlength*2*2);
    memset(prx_buffer,0,txlength*2*2);     
    //  ad9361_set_bb_rate(phydev,1111); 
 
   printf("init buffer finish\r\n");

printf("init white noisy data\r\n");
  char *whitenoisyfilename = "test.iq";
 int fdi = open(whitenoisyfilename,  O_RDONLY);
     printf("read white noisy data finish:%d\r\n",fdi);
     int bsize = 4096;
    int16_t *whitenoisyiqbuf = (int16_t*)malloc(2*bsize*sizeof(int16_t));
    read(fdi, whitenoisyiqbuf, 2*bsize*sizeof(int16_t));
    close(fdi);
printf("init ofdm data\r\n");
   char *OFDMfilename = "OFDM.bin";
 int fdiOFDM = open(OFDMfilename,  O_RDONLY);
     printf("read ofdm data finish:%d\r\n",fdi);
    bsize = 4096;
    int16_t *OFDMiqbuf = (int16_t*)malloc(2*bsize*sizeof(int16_t));
    read(fdiOFDM, OFDMiqbuf, 2*bsize*sizeof(int16_t));
    close(fdiOFDM);

    printf("init BPSK data\r\n");
   char *BPSKfilename = "BPSK.bin";
 int fdiBPSK = open(BPSKfilename,  O_RDONLY);
     printf("read BPSK data finish:%d\r\n",fdiBPSK);
    bsize = 4096;
    int16_t *BPSKiqbuf = (int16_t*)malloc(2*bsize*sizeof(int16_t));
    read(fdiBPSK, BPSKiqbuf, 2*bsize*sizeof(int16_t));
    close(fdiBPSK);

printf("init QPSK data\r\n");
   char *QPSKfilename = "QPSK.bin";
 int fdiQPSK = open(QPSKfilename,  O_RDONLY);
     printf("read QPSK data finish:%d\r\n",fdiQPSK);
    bsize = 4096;
    int16_t *QPSKiqbuf = (int16_t*)malloc(2*bsize*sizeof(int16_t));
    read(fdiQPSK, QPSKiqbuf, 2*bsize*sizeof(int16_t));
    close(fdiQPSK);

  if(bandwidth_khz >56000){
        bandwidth_khz = 56000;
    }



   //  printf("read ofdmdataq finish:%d\r\n",length);
    // txcfg.bw_hz = KHZ(bandwidth_khz);
    txcfg.bw_hz = MHZ(56); // 3.0 MHz RF bandwidth
    txcfg.fs_hz = MHZ((61.44));   // 2.5 MS/s tx sample rate
    txcfg.lo_hz = GHZ(2.4); // 1.57542 GHz RF frequency
    txcfg.rfport = "B";
    txcfg.gain_db = 0.0;
    txcfg.ispoweroff = true;

    rxcfg.bw_hz = MHZ(56); // 3.0 MHz RF bandwidth
    rxcfg.fs_hz = MHZ((61.44));   // 2.5 MS/s tx sample rate
    rxcfg.lo_hz = GHZ(2.4); // 1.57542 GHz RF frequency
    rxcfg.rfport = "B_BALANCED";
    rxcfg.gain_db = 0.0;
    rxcfg.ispoweroff = true;
    char ip[32] = {0};

    get_local_ip(IF_NAME, ip);

    if(0 != strcmp(ip, ""))
        printf("%s ip is %s\n",IF_NAME, ip);
   printf("* Acquiring IIO context\r\n");
   ASSERT((ctx = iio_create_network_context(ip)) && "No context");
	ASSERT(iio_context_get_devices_count(ctx) > 0 && "No devices");

    printf("* Acquiring devices\r\n");
    int device_count = iio_context_get_devices_count(ctx);
    if (!device_count) {
       printf("No supported SDR devices found.\n");
        return -1;
    }
  printf("* Context has %d device(s).\r\n",device_count);

 printf("* Acquiring AD9361 streaming devices\n");
	ASSERT(get_ad9361_stream_dev(ctx, TX, &tx) && "No tx dev found");
	ASSERT(get_ad9361_stream_dev(ctx, RX, &rx) && "No rx dev found");

	printf("* Configuring AD9361 for streaming\n");
	ASSERT(cfg_ad9361_streaming_ch(ctx, &rxcfg, RX, 0) && "RX port 0 not found");
	ASSERT(cfg_ad9361_streaming_ch(ctx, &txcfg, TX, 0) && "TX port 0 not found");

	printf("* Initializing AD9361 IIO streaming channels\n");
	ASSERT(get_ad9361_stream_ch(ctx, RX, rx, 0, &rx0_i) && "RX chan i not found");
	ASSERT(get_ad9361_stream_ch(ctx, RX, rx, 1, &rx0_q) && "RX chan q not found");
	ASSERT(get_ad9361_stream_ch(ctx, TX, tx, 0, &tx0_i) && "TX chan i not found");
	ASSERT(get_ad9361_stream_ch(ctx, TX, tx, 1, &tx0_q) && "TX chan q not found");

	printf("* Enabling IIO streaming channels\n");
	iio_channel_enable(rx0_i);
	iio_channel_enable(rx0_q);
	iio_channel_enable(tx0_i);
	iio_channel_enable(tx0_q);
      phydev = iio_context_find_device(ctx, "ad9361-phy");
 struct iio_channel* phy_chn = iio_device_find_channel(phydev, "voltage0", true);
	printf("* Creating non-cyclic IIO buffers with 1 MiS\n");
	rx_buffer = iio_device_create_buffer(rx, txlength, false);
	if (!rx_buffer) {
		perror("Could not create RX buffer");
		shuTdown();
	}
	tx_buffer = iio_device_create_buffer(tx, bsize, true);
	if (!tx_buffer) {
		perror("Could not create TX buffer");
		shuTdown();
	}
     iio_channel_attr_write_double(phy_chn, "hardwaregain", txcfg.gain_db);
     iio_channel_attr_write_double(phy_chn, "hardwaregain", rxcfg.gain_db);
      iio_channel_attr_write_bool(
                iio_device_find_channel(phydev, "altvoltage0", true)
              , "powerdown", false); // Turn OFF RX LO
               iio_channel_attr_write_bool(
                iio_device_find_channel(phydev, "altvoltage1", true)
              , "powerdown", false); // Turn OFF RX LO
	prx_buffer = (short *)iio_buffer_start(rx_buffer);
	ptx_buffer = (short *)iio_buffer_start(tx_buffer);
    for(int i=0;i<bsize;i++){
                ptx_buffer[i * 2] =  OFDMiqbuf[2*i];//fill I data
                ptx_buffer[i * 2+1]= OFDMiqbuf[2*i+1];//*(ofdmq_tmp+i) ;//fill q data
    }
            int   ntx = iio_buffer_push(tx_buffer);
	printf("* Starting IO streaming (press CTRL+C to cancel)\n");
            
    int32_t server_sockfd;
    ssize_t len;
  
    struct sockaddr_in my_addr;   //服务器网络地址信息结构体
    struct sockaddr_in remote_addr; //客户端网络地址结构体
    uint32_t sin_size;
    uint8_t buf[MAXDATASIZE];  //数据传送的缓冲区

    memset(&my_addr, 0,sizeof(my_addr)); //数据初始化--清零
    my_addr.sin_family = AF_INET; //设置为IP通信
    my_addr.sin_addr.s_addr = INADDR_ANY;//服务器IP地址--允许连接到所有本地地址上
    my_addr.sin_port = htons(port_in); //服务器端口号

    /*创建服务器端套接字--IPv4协议，面向无连接通信，UDP协议*/
    if((server_sockfd = socket(PF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket error");
        return;
    }
  

	
	
	

    /*将套接字绑定到服务器的网络地址上*/
    if (bind(server_sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) < 0)
    {
        perror("bind error");
        return;
    }
    sin_size = sizeof(struct sockaddr_in);
    printf(" waiting for a packet...\n");
   
    
    int powerctt = 0;
    
    int mainpowerctt = 0;
    
    *gpio  = 0x00;
    /*接收客户端的数据并将其发送给客户端--recvfrom是无连接的*/
    while (((len = recvfrom(server_sockfd, buf, MAXDATASIZE, 0, (struct sockaddr *)&remote_addr, &sin_size)) > 0)) {
      
      
      printf(" received packet from:%s\n",inet_ntoa(remote_addr.sin_addr));
           printf("%c\r\n",buf[0]);
            // sendto(server_sockfd,buf,strlen(buf),0,(struct sockaddr*)&remote_addr,sizeof(remote_addr));
      if(buf[0] =='$')
	{
  	   
               char header[10] ;
             
               double gaindb;
               double start;
               
               double rxfrequence;

               
               sscanf(buf,"$,%lf,%lf,%d,%lf,%d,%d,%d,%d,%d",
               &start,&rxfrequence,&powerctt,&gaindb,&wavelength,&capturetimes,&Countdowntime,&maxdelaytime,&isContinuouslaunch);
	          printf("gaindb:%lf,start:%lf,powerctt:%d,rxfrequence:%lf\r\n",gaindb,start,powerctt,rxfrequence);
	if(strstr(buf,"ON"))//signal source ON
	{
       *(radarparmctrl) = wavelength;
       *(radarparmctrl+1) = capturetimes;
       *(radarparmctrl+2) = Countdowntime;
       *(radarparmctrl+3) = maxdelaytime;
       *(radarparmctrl+4) = isContinuouslaunch;

       oldwavelength = wavelength;
       oldcapturetimes = capturetimes;
       oldCountdowntime = Countdowntime;
       oldmaxdelaytime = maxdelaytime;
       oldisContinuouslaunch = isContinuouslaunch;


    iio_channel_attr_write_bool(
                 iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "altvoltage1", true)
                    , "powerdown", true); // Turn OFF TX LO
		if(powerctt)
		{
                *gpio = 0x00;
		mainpowerctt = powerctt;
		}
		// parse frequence
              	double frequence = start;
		// set rx frequence
		if(rxcfg.lo_hz!= MHZ(rxfrequence))
		{
		
		rxcfg.lo_hz = MHZ(rxfrequence);
		
		printf("rxfrequence:%d,\r\n",rxfrequence);
		iio_channel_attr_write_longlong(
                iio_device_find_channel(phydev, "altvoltage0", true)
                , "frequency",rxcfg.lo_hz); // Set RX LO frequency	
		}
		
 		if(txcfg.gain_db != gaindb)
    		{
			iio_channel_attr_write_double(phy_chn, "hardwaregain", gaindb);
			usleep(100000);
			txcfg.gain_db = gaindb;
		}
		if(txcfg.lo_hz != MHZ(frequence))
  		{
         
                       iio_channel_attr_write_longlong(
                iio_device_find_channel(phydev, "altvoltage0", true)
                , "frequency", MHZ(frequence)); // Set TX LO frequency	
		usleep(100000);  
		txcfg.lo_hz = MHZ(frequence);
		} 
           iio_channel_attr_write_bool(
                 iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "altvoltage1", true)
                    , "powerdown", false); // Turn OFF TX LO 
		if(powerctt)
		{
                *gpio = 0xFF;
		mainpowerctt = powerctt;
		}
		printf("open power signal source ...............\r\n");	

		}else if(strstr(buf,"OFF"))//signal source OFF
		{
            // repeat_stop = true;
			   printf("close device\r\n");
               		   currentswitchstate = 0;
		      
			      iio_channel_attr_write_bool(
                    iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "altvoltage1", true)
                    , "powerdown", true); // Turn OFF TX LO
			   usleep(100000);
 			   printf("close power\r\n");
		if(!powerctt)
		{
               		*gpio = 0x00;
			mainpowerctt = powerctt;
		}
			
			usleep(100000);
		
	}//CK
            else if(strstr(buf,"CHECK"))
            {
		printf("check device00\r\n");	
                char data[MAXDATASIZE] ;
                long long getbandwidth = 0;
 		long long getlo = 0,getRXlo;
		bool power;
		//
        iio_channel_attr_read_longlong(
                iio_device_find_channel(phydev, "altvoltage0", true)
                , "frequency", &getRXlo);
 		 iio_channel_attr_read_longlong(
                iio_device_find_channel(phydev, "altvoltage1", true)
                , "frequency", &getlo);
		double getgainvalue=-1.00;
		printf("check device1\r\n");	
		iio_channel_attr_read_double(phy_chn, "hardwaregain", &getgainvalue);
		printf("check device2\r\n");	
		iio_channel_attr_read_bool(
                	  iio_device_find_channel( iio_context_find_device(ctx, "ad9361-phy"), "altvoltage1", true)
               			 , "powerdown", &power); // Turn OFF RX LO
		printf("check device3\r\n");	
		double lo = getlo/1000000.00;
		double rxlo = getRXlo/1000000.00;
		if(!power)
		{
		sprintf(data,"#,%lf,%lf,%d,%lf,%d,%d,%d,%d,%d,ON,#END",lo,rxlo,mainpowerctt,getgainvalue,oldwavelength,oldcapturetimes,oldCountdowntime,oldmaxdelaytime,oldisContinuouslaunch);

                sendto(server_sockfd,data,strlen(data),0,(struct sockaddr*)&remote_addr,sizeof(remote_addr));
		}else
		{
		sprintf(data,"#,%lf,%lf,%d,%lf,%d,%d,%d,%d,%d,OFF,#END",lo,rxlo,mainpowerctt,getgainvalue,oldwavelength,oldcapturetimes,oldCountdowntime,oldmaxdelaytime,oldisContinuouslaunch );

                sendto(server_sockfd,data,strlen(data),0,(struct sockaddr*)&remote_addr,sizeof(remote_addr));
		}
			printf("check device  success!\r\n");	
            }

		

		
             
	      
	}   
    /*关闭套接字*/
    
    }

	      close(server_sockfd);
        printf("release source fd=%d\n",server_sockfd);
       munmap(gpio, getpagesize() * 250);
        close(fd);
    
       sleep(1);
   
	return 0;
}

void * repeat_transfer(void * arg)
{
    // RX and TX sample counters
	
 
    // while (1)
    // {
    // usleep(100000);
    //     /* code */
   
    // if (!repeat_stop)
	// {
       
	
	// 	// // Refill RX buffer
	// 	// int nbytes_rx = iio_buffer_refill(rx_buffer);
      
	// 	// memcpy(ptx_buffer, prx_buffer, nbytes_rx);

	// 	// // Schedule TX buffer
    //     //  iio_buffer_push(tx_buffer);
		
    // }else{

    // }
	// }
        return NULL;
}

