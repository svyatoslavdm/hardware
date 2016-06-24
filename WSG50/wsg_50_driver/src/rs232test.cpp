#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>

#include "wsg_50/common.h"
#include "wsg_50/checksum.h"
#include "wsg_50/msg.h"
#include "wsg_50/cmd.h"
#include "wsg_50/functions.h"

#include "ros/ros.h"


/* установки скорости передачи данных объявлены в <asm/termbits.h>, 
который включается в <termios.h> */
#define BAUDRATE B38400

/* измените эти объявления для корректного указания порта */
#define RSDEVICE "/dev/ttyUSB2"
//#define _POSIX_SOURCE 1 /* POSIX-совместимый источник */

#define FALSE 0
#define TRUE 1

volatile int STOP=FALSE;

//rs232_close()
//{
//    close(fd);
//}

main()
{
//    std::string device_s = "/dev/ttyUSB2";
//    const char * device = device_s.c_str();
//    unsigned int bitrate = 38400;
//    ros::Time::init();

//    int res_connect = cmd_connect_serial(device, bitrate);

//    printf("That's it!");

//    for (int i = 0; i < 100; i++)
//    {
//	    gripper_response info;
//	    float acc = 0.0;
//	    info.speed = 0.0;
//	
//        const char * state = systemState();
//            if (!state)
//                return 0;
//            info.state_text = std::string(state);
//		    info.position = getOpening();
//		    acc = getAcceleration();
//		    info.f_motor = getForce();
//		
//	    printf("%f, \n", info.f_motor);
//        ros::Duration(1).sleep();
//    }             	
//    printf("That's it!");
//    
//    return 0;
    
    
    
    
    
    
    
    
    
    
    
//    rs232_open()
    int fd;
    struct termios oldtio,newtio;
    unsigned char *buf;
    /* 
    Открываем устройство модема для чтения и записи как неуправляющий
    терминал (tty), поскольку мы не хотим завершать процесс когда
    помехи в линии посылают сигнал CTRL-C.
    */
    fd = open(RSDEVICE, O_RDWR | O_NOCTTY ); 
    if (fd < 0) 
    {
        perror(RSDEVICE);
        exit(-1); 
    }

    tcgetattr(fd,&oldtio); /* save current serial port settings */
    bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

    /* 
    BAUDRATE: устанавливает скорость передачи данных в bps.
              Вы также могли бы использовать cfsetispeed и cfsetospeed.
    CRTSCTS : аппаратное управление выводным потоком данных (используется
              только если кабель обладает всеми необходимыми сигнальными
              линиями. См. секцию 7 Serial-HOWTO)
    CS8     : 8n1 (8 бит, без четности, 1 стоп-бит)
    CSTOPB  : стоп-бит
    CLOCAL  : локальное подключение, нет управления модемом
    CREAD   : разрешает прием символов
    */

    newtio.c_cflag = BAUDRATE | CS8 | CSTOPB | CREAD | CLOCAL;

//    newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;

    /*
    IGNBRK  : игнорировать завершение
    IGNPAR  : игнорировать байты с ошибками четности
    ICRNL   : отобразить символ CR на NL (иначе ввод CR на другом
              компьютере не будет завершать ввод)
              иначе установить устройство как raw (нет обработки ввода)
    */
    newtio.c_iflag = IGNBRK | ICRNL;

    /*
    Raw вывод (нет обработки вывода).
    */
    newtio.c_oflag = 0;

    /*
    ICANON  : разрешить канонический ввод
    заблокировать любое функционирование "эхо", и не посылать сигналы
    к вызывающей программе
    */
    newtio.c_lflag = 0;

    /* 
    инициализация всех управляющих символов
    значения по умолчанию могут быть найдены в in /usr/include/termios.h,
    и даются в комментариях, но мы не нуждаемся в них здесь
    */
    newtio.c_cc[VTIME]     = 0;     /* посимвольный таймер не используется */


    /* 
    теперь очищаем модемную линию и активируем наши установки порта
    */
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);
   
//    rs232_read()

    int res;
    while (true)    
//    for (int i = 0; i < 50; i++)
    {
        res = read(fd, &buf, 255);   // buf, len - ??
        
	    if ( res < 0 )
	    {
		    fprintf( stderr, "Failed to read from serial device\n" );
    //		exit(1);
	    }
    //    return res;
        
//        buf[res]=0;             /* установить завершитель строки,
//	                               таким образом, мы можем выполнять
//	                               printf */
//	                               
//        printf(":%s:%d\n", buf, res);

//        std::cout << std::ios::hex << (unsigned char)buf[0] << " " <<
//                     std::ios::hex << (unsigned char)buf[1] << " " <<
//                     std::ios::hex << (unsigned char)buf[2] << " " <<
//                     std::ios::hex << (unsigned char)buf[3] << std::endl;
        printf("%X\n", *buf);
    }    
//    rs232_write()
    
//    res = write(fd, buf, 255);   // buf, len - ??


	unsigned char payload[1];

	// Set flags: Homing direction (0: default, 1: widthitive movement, 2: negative movement).
	payload[0] = 0x00;

	// Submit command and wait for response. Push result to stack.
	// Assemble message struct
	msg_t msg =
	{
		.id = 0x20,
		.len = 1,
		.data = payload
	};

	// Send command

	unsigned char header[MSG_PREAMBLE_LEN + 3];
    //unsigned char checksum[2];
	unsigned short crc;

	// Preamble
	for ( int i = 0; i < MSG_PREAMBLE_LEN; i++ ) header[i] = MSG_PREAMBLE_BYTE;

	// Command ID
	header[MSG_PREAMBLE_LEN] = msg.id;

	// Length
	header[MSG_PREAMBLE_LEN + 1] = lo( msg.len );
	header[MSG_PREAMBLE_LEN + 2] = hi( msg.len );

	// Checksum
	crc = checksum_crc16( header, 6 );
	crc = checksum_update_crc16( msg.data, msg.len, crc );

	unsigned char * buf_ = (unsigned char *)malloc( 6 + msg.len + 2 ); // 6+2 fixes (PREAMBLE, ID, PAILOAD / ... / CRC)
	
//    unsigned char buf_[6 + msg.len + 2];
//    unsigned char *pbuf_ = &buf_;
//    for ( int i = 0; i < 6; i++ )
//        buf_[i] = header[i];
//    for ( int i = 0; i < msg.len; i++ )
//        buf_[6 + i] = msg.data[i];
//    for ( int i = 0; i < 2; i++ )
//        buf_[6 + msg.len + i] = &crc;
	memcpy( buf_, header, 6 );
	memcpy( buf_ + 6, msg.data, msg.len );
	memcpy( buf_ + 6 + msg.len, (unsigned char *) &crc, 2 );

    res = write( fd, buf_, 6 + msg.len + 2);   // buf, len - ??
    if ( res < 6 + (int)msg.len + 2 )
	{
		quit( "Failed to submit message checksum" );
	}

	free( buf_ );


    printf( "Finished\n" );
    /* восстановление старых установок порта */
    tcsetattr(fd,TCSANOW,&oldtio);
}
