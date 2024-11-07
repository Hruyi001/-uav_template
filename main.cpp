/*
 * Copyright (C) 2020 Rockchip Electronics Co., Ltd.
 * Authors:
 *  PutinLee <putin.lee@rock-chips.com>
 *  Cerf Yu <cerf.yu@rock-chips.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <math.h>
#include <fcntl.h>
#include <memory.h>
#include <unistd.h>
#include <sys/time.h>
#include <string.h> 
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <errno.h>
#include <termios.h>
#include <pthread.h>
#include <sys/stat.h>
#include <dirent.h>
// #include <opencv2/opencv.hpp>
//#include "im2d.hpp"
//#include "RockchipRga.h"
//#include "RgaUtils.h"
//#include "rga.h"
//#include "args.h"
#include "template1.h"

#define WIDTH 1920
#define HEIGHT 1080
#define BUFFER_COUNT 40

#define ERROR               -1

/********** SrcInfo set **********/
#define SRC_WIDTH  1920
#define SRC_HEIGHT 1080

#define SRC_FORMAT RK_FORMAT_YCbCr_420_SP

/********** DstInfo set **********/
#define DST_WIDTH  1920
#define DST_HEIGHT 1080

#define DST_FORMAT RK_FORMAT_RGB_888

#define UART_DEVICE "/dev/ttyS7"
#define BAUD_RATE B115200
#define FRAME_SIZE 48

// 定义帧头、帧尾和其他常量
#define FRAME_HEAD1 0xEB
#define FRAME_HEAD2 0x90
#define FRAME_TAIL  0xAA

float a1,a2,a3,a4,a5,a6,a7;
double a8,a9;

int uart_fd;
pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;


// 计算校验和，从偏移量4到19的所有字节的累加和，取低字节
unsigned char calculateChecksum(unsigned char* data, size_t length) {
    unsigned int sum = 0;
    for (size_t i = 4; i < length; ++i) {
        sum += data[i];
    }
    return sum & 0xFF;  // 取低字节
}
// 发送帧数据
void sendFrame(int uart_fd, unsigned char* frame, size_t frameLength) {

    if (uart_fd != -1) {
        // printf("帧长度%d\n", frameLength);
        // printf("%s\n", frame);
        write(uart_fd, frame, frameLength);
    } else {
        perror("串口未打开");
    }
} 
   
void process_frame(unsigned char *frame, int frame_len) {
    // Add your frame processing logic here
    // printf("Received frame: ");
    // for (int i = 0; i < frame_len; i++) {
    //     printf("%02X ", frame[i]);
    // }
        // printf(" frame num:%d ",frame[1]);
    // 0xaa开始，0x55结束
    if(frame[0] == 0xaa && frame[47] == 0x55 && (!(frame[1]%10)))
    {

        pthread_mutex_lock(&mutex1);

        memcpy(&a1,&frame[2],sizeof(float));
        memcpy(&a2,&frame[6],sizeof(float));
        memcpy(&a3,&frame[10],sizeof(float));
        memcpy(&a4,&frame[14],sizeof(float));
        memcpy(&a5,&frame[18],sizeof(float));
        memcpy(&a6,&frame[22],sizeof(float));
        memcpy(&a7,&frame[26],sizeof(float));
        memcpy(&a8,&frame[30],sizeof(double));
        memcpy(&a9,&frame[38],sizeof(double));
        // printf("a1:%f a2:%f a3:%f a4:%f a5:%f a6:%f a7:%f a8:%f a9:%f\r\n",a1,a2,a3,a4,a5,a6,a7,a8,a9);

        pthread_mutex_unlock(&mutex1);
    }
        
    // printf("\n");
}

void *uart_thread(void *arg) {
 unsigned char buf[FRAME_SIZE];
    int buf_pos = 0;
    int frame_start = 0;

    while (1) {
        unsigned char byte;
        int n = read(uart_fd, &byte, 1);
        if (n > 0) {
            if (frame_start) {
                buf[buf_pos++] = byte;
                if (buf_pos == FRAME_SIZE) {
                    process_frame(buf, FRAME_SIZE);
                    buf_pos = 0;
                    frame_start = 0;
                }
            } else {
                if (byte == 0xAA) {
                    frame_start = 1;
                    buf[buf_pos++] = byte;
                }
            }
        } else if (n < 0) {
            perror("Read error");
        }
    } 
    return NULL;
}

int configure_uart(const char *device) {
    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        perror("Unable to open UART");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);

    // Set baud rate
    cfsetispeed(&options, BAUD_RATE);
    cfsetospeed(&options, BAUD_RATE);

    // Configure 8 data bits, no parity, 1 stop bit
    options.c_cflag &= ~PARENB;    // No parity
    options.c_cflag &= ~CSTOPB;    // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;        // 8 data bits

    // Enable the receiver and set local mode
    options.c_cflag |= (CLOCAL | CREAD);

    // Set raw input (non-canonical) mode
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    // Set read timeout
    options.c_cc[VMIN] = FRAME_SIZE;  // Minimum number of characters to read
    options.c_cc[VTIME] = 0; // Timeout in deciseconds (0 for non-blocking)

    tcsetattr(fd, TCSANOW, &options);

    return fd;
}

int count_subdirectories(const char *path) {
    DIR *dir;
    struct dirent *entry;
    int count = 0;

    dir = opendir(path);
    if (dir == NULL) {
        perror("opendir");
        return -1;
    }

    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type == DT_DIR) {
            if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0) {
                count++;
            }
        }
    }

    closedir(dir);
    return count;
}
void saveYUV420(const cv::Mat& yuvImg, const std::string& filename) {
    // 打开文件
    std::ofstream file(filename, std::ios::out | std::ios::binary);

    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }

    // 确定图片的宽度和高度
    int width = yuvImg.cols;
    int height = yuvImg.rows;

    // 获取 Y 分量
    file.write(reinterpret_cast<const char*>(yuvImg.data), width * height);

    // 获取 U 分量和 V 分量，假设 U 和 V 分量紧跟在 Y 之后
    file.write(reinterpret_cast<const char*>(yuvImg.data + width * height), width * height / 4); // U 分量
    file.write(reinterpret_cast<const char*>(yuvImg.data + width * height * 5 / 4), width * height / 4); // V 分量

    file.close();
}

int main(int argc, char*  argv[]) {
    int ret = 0;
    int MODE;
    // IM_STATUS         STATUS;

    // im_rect         src_rect;
    // im_rect         dst_rect;
    // rga_buffer_t     src;
    // rga_buffer_t     dst;
    // rga_buffer_handle_t src_handle;
    // rga_buffer_handle_t dst_handle;

    int fd;
    struct v4l2_capability cap;
    struct v4l2_format fmt;
    struct v4l2_requestbuffers req;
    struct v4l2_buffer buf;
    struct v4l2_plane planes[1];
    void *buffers[BUFFER_COUNT];
    int i;

    char* src_buf = NULL;
    char* dst_buf = NULL;

    const char *path = "/mnt";
    // int num_subdirs = count_subdirectories(path);

    // if (num_subdirs < 0) {
    //     fprintf(stderr, "Error counting subdirectories in %s\n", path);
    //     return 1;
    // }

    // char new_dir[15];

    // sprintf(new_dir,"/mnt/%d",num_subdirs);

    // if (mkdir(new_dir, 0777) == -1) {
    //     perror("mkdir");
    //     return 1;
    // }

    // printf("Created directory: %s\n", new_dir);

    uart_fd = configure_uart(UART_DEVICE);
    if (uart_fd == -1) {
        return EXIT_FAILURE;
    }

    pthread_t thread_id;
    if (pthread_create(&thread_id, NULL, uart_thread, NULL) != 0) {
        perror("Failed to create thread");
        close(uart_fd);
        return EXIT_FAILURE;
    }


    // memset(&src_rect, 0, sizeof(src_rect));
    // memset(&dst_rect, 0, sizeof(dst_rect));
    // memset(&src, 0, sizeof(src));
    // memset(&dst, 0, sizeof(dst));

    // a1=0;
    // a2=0;
    // a3=0;
    // a4=0;
    // a5=0;
    // a6=0;
    // a7=0;
    // a8=0;
    // a9=0;
        /********** Get parameters **********/

    // src_buf = (char*)malloc(SRC_WIDTH*SRC_HEIGHT*get_bpp_from_format(SRC_FORMAT));  //申请src_buf内存
    // dst_buf = (char*)malloc(DST_WIDTH*DST_HEIGHT*get_bpp_from_format(DST_FORMAT));  //申请dst_buf内存


     // 打开设备
    fd = open("/dev/video0", O_RDWR);
    if (fd == -1) {
        perror("打开设备失败");
        exit(EXIT_FAILURE);
    }

    // 查询设备能力
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {   
        perror("查询设备失败");
        close(fd);
        exit(EXIT_FAILURE);
    }

    // 设置格式
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width = WIDTH;
    fmt.fmt.pix_mp.height = HEIGHT;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
    fmt.fmt.pix_mp.num_planes = 1;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
        perror("设置格式失败");
        close(fd);
        exit(EXIT_FAILURE);
    }

    // 请求缓冲区
    memset(&req, 0, sizeof(req));
    req.count = BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
        perror("请求缓冲区失败");
        close(fd);
        exit(EXIT_FAILURE);
    }

    // 映射缓冲区
    for (i = 0; i < BUFFER_COUNT; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.m.planes = planes;
        buf.length = 1;
        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
            perror("查询缓冲区失败");
            close(fd);
            exit(EXIT_FAILURE);
        }
        printf("buffers[i]:%d buf.m.planes[0].length:%d buf.m.planes[0].m.mem_offset:%d\r\n",i,buf.m.planes[0].length,buf.m.planes[0].m.mem_offset);
        buffers[i] = mmap(NULL, buf.m.planes[0].length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.planes[0].m.mem_offset);
        if (buffers[i] == MAP_FAILED) {
            perror("映射缓冲区失败");
            close(fd);
            exit(EXIT_FAILURE);
        }
    }

    // 开始采集
    for (i = 0; i < BUFFER_COUNT; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.m.planes = planes;
        buf.length = 1;
        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            perror("入队列失败");
            close(fd);
            exit(EXIT_FAILURE);
        }
    }
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) {
        perror("开始采集失败");
        close(fd);
        exit(EXIT_FAILURE);
    }

    //     //为输入输出图像buf，申请虚拟缓冲区
    // src_handle = importbuffer_virtualaddr(src_buf, SRC_WIDTH, SRC_HEIGHT, SRC_FORMAT);
    // if (src_handle <= 0) {
    //     printf("Failed to import virtualaddr for src channel!\n");
    //     return ERROR;
    // }
    // dst_handle = importbuffer_virtualaddr(dst_buf, DST_WIDTH, DST_HEIGHT, DST_FORMAT);
    // if (dst_handle <= 0) {
    //     printf("Failed to import virtualaddr for dst channel!\n");
    //     return ERROR;
    // }

    // src = wrapbuffer_handle(src_handle, SRC_WIDTH, SRC_HEIGHT, SRC_FORMAT);
    // dst = wrapbuffer_handle(dst_handle, DST_WIDTH, DST_HEIGHT, DST_FORMAT);
    // if(src.width == 0 || dst.width == 0) {
    //     printf("%s, %s", __FUNCTION__, imStrError());
    //     return ERROR;
    // }

    // src.format = RK_FORMAT_YCbCr_420_SP;
    // dst.format = RK_FORMAT_RGB_888;

	unsigned char rgbdata[WIDTH*HEIGHT*3];
    int addnum =0;
    // char filenameyuv[100];
    // char filenameDH[100] = " ";
    // char filename[100] = "DH-Data.txt";

    // char dh_data[1024];
    // char timestrfile[50];
    // 创建一个字节数组来保存帧数据
    unsigned char frame[22];
    // sprintf(filenameDH,"%s/%s",new_dir,filename);
    // int fpdh = open(filenameDH, O_WRONLY | O_CREAT | O_APPEND, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    // if (fpdh == -1) {
    //     printf("%s\r\n",filenameDH);
    //         perror("打开dh文件失败");
    // exit(EXIT_FAILURE);
    // }
    // 采集帧数据
    std::pair<double, double> last_gps = {109.22046882, 38.56923820};
     while(1) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.m.planes = planes;
        buf.length = 1;
        if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
            if (errno == EAGAIN) {
                continue; // 当缓冲区为空时，重试
            } else {
                perror("出队列失败");
                close(fd);
                exit(EXIT_FAILURE);
            }
        }

        if((addnum%20) == 0){

            // memcpy(src_buf,buffers[buf.index],WIDTH * HEIGHT * 3 / 2);   //将源文件数据  读取到src_buf
            // STATUS = imcvtcolor(src, dst, src.format, dst.format);
            // 将yuv图像转成bgr图像
            // char *src = char*(buffers[buf.index]);
            
            // cvtColor(src, tpl, COLOR_YUV2BGR_I420);
            cv::Mat tpl;
            int width = 1920; // 替换为你的实际宽度
            int height = 1080; // 替换为你的实际高度
            // tpl = cv::Mat(height, width, CV_8UC3);
            // 创建一个包含 YUV 数据的 Mat 对象
            cv::Mat yuvImg(height + height / 2, width, CV_8UC1, buffers[buf.index]);
            // saveYUV420(yuvImg, "./yuv/out.yuv");
            // 转换为 BGR 图像
            cv::Mat bgrImg; 
           // cv::cvtColor(yuvImg, tpl, cv::COLOR_YUV2BGR_I420);
            cv::cvtColor(yuvImg, tpl, cv::COLOR_YUV2GRAY_I420); 
         //   std::string path = "/mnt/11/" + std::to_string(addnum) + ".jpg";
            //imwrite(path, tpl);
            // // tpl = imread("./03.jpg");
            // Mat target = imread("9.2.tif");

            
            // 调用算法的代码
            double angle = -15;
            auto start = std::chrono::high_resolution_clock::now(); // Start timing
            auto ioc = template_demo(tpl, angle, last_gps);
            auto end = std::chrono::high_resolution_clock::now(); // End timing
            std::chrono::duration<double, std::milli> elapsed = end - start; // Calculate elapsed time
            std::cout << "Algorithm execution time: " << elapsed.count() << " ms" << std::endl; // Output time
            last_gps = ioc;



            // 帧传输
            double longitude = ioc.first;
            double latitude = ioc.second; 
            // double longitude = 37.12321432;
            // double latitude = 192.15345453; 
            // printf("%.8f, %.8f\n", longitude, latitude);
            // 填充帧头
            frame[0] = FRAME_HEAD1;
            frame[1] = FRAME_HEAD2;

            // 数据长度（固定为16字节）
            unsigned short dataLength = 16;
            frame[2] = (dataLength >> 8) & 0xFF;  // 高字节
            frame[3] = dataLength & 0xFF;         // 低字节

            // 将double类型的经度和纬度转换为字节数组（大端字节序）
            unsigned char* lonBytes = (unsigned char*)&longitude;
            unsigned char* latBytes = (unsigned char*)&latitude;

            for (int i = 0; i < 8; ++i) {
                // frame[4 + i] = lonBytes[7 - i];  // 大端序
                // frame[12 + i] = latBytes[7 - i]; // 大端序
                frame[4 + i] = lonBytes[i];  // 大端序
                frame[12 + i] = latBytes[i]; // 大端序v
            }

            // 计算并填充校验位
            frame[20] = calculateChecksum(frame, 20);

            // 填充帧尾
            frame[21] = FRAME_TAIL;  

            // 发送帧数据
            sendFrame(uart_fd, frame, sizeof(frame)); 
            printf("数据发送完毕！\n");  
            // struct timeval tv;
            // gettimeofday(&tv, NULL);
            // time_t sec = tv.tv_sec;
            // struct tm *local_time = localtime(&sec);

            // sprintf(timestrfile,"%d-%02d-%02d-%02d:%02d:%02d.%03ld",
            // local_time->tm_year + 1900, local_time->tm_mon + 1, local_time->tm_mday,
            // local_time->tm_hour, local_time->tm_min, local_time->tm_sec,
            // tv.tv_usec / 1000);

            // snprintf(filenameyuv, sizeof(filenameyuv), "%s/%s-frame.rgb",new_dir,timestrfile);
            // int fpyuv = open(filenameyuv, O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
            // if (fpyuv == -1) {
            // perror("打开yuv文件失败");
            // close(fd);
            // exit(EXIT_FAILURE);
            // }

            // pthread_mutex_lock(&mutex);
            // sprintf(dh_data, "%s ROLL:%f PITCH:%f YAW:%f VN:%f VE:%f VD:%f ALT:%f LON:%lf LAT:%lf\r\n",timestrfile,a1,a2,a3,a4,a5,a6,a7,a8,a9);

            // // printf(" dh data:%s\r",dh_data);
            // pthread_mutex_unlock(&mutex);

            // write(fpdh,dh_data,strlen(dh_data));

            // write(fpyuv, dst_buf, WIDTH * HEIGHT * 3); // 保存 yuv 分量

            // close(fpyuv);
        }

        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            perror("入队列失败");
            close(fd);
            exit(EXIT_FAILURE);
        }
        addnum++;
    }

    pthread_join(thread_id, NULL);
    close(uart_fd);
    // close(fpdh);

        //释放输入输出buf
    // releasebuffer_handle(src_handle);        
    // releasebuffer_handle(dst_handle);

    // if (src_buf != NULL) {
    //     free(src_buf);
    //     src_buf = NULL;
    // }

    // if (dst_buf != NULL) {
    //     free(dst_buf);
    //     dst_buf = NULL;
    // }


    // 停止采集
    if (ioctl(fd, VIDIOC_STREAMOFF, &type) == -1) {
        perror("停止采集失败");
        close(fd);
        exit(EXIT_FAILURE);
    }

    // 解除映射
    for (i = 0; i < BUFFER_COUNT; i++) {
        if (munmap(buffers[i], buf.m.planes[0].length) == -1) {
            perror("解除映射失败");
        }
    }

    close(fd);

    return 0;
}

