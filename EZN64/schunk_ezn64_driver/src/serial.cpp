#include "schunk_ezn64_driver/serial.h"

#define BAUDRATE B9600
#define FALSE 0
#define TRUE 1
#define hi( x )    	(unsigned char) ( ((x) >> 8) & 0xff )	// Returns the upper byte of the passed short
#define lo( x )    	(unsigned char) ( (x) & 0xff )       	// Returns the lower byte of the passed short

using namespace std;

volatile int STOP = FALSE;

Serial::Serial(string rsdevice)
{
    this->RSDEVICE = rsdevice;
    this->fd = -1;
};

Serial::~Serial()
{
};

int Serial::Connect()
{
//    Открываем устройство модема для чтения и записи как неуправляющий
//    терминал (tty), поскольку мы не хотим завершать процесс когда
//    помехи в линии посылают сигнал CTRL-C.

    this->fd = open(this->RSDEVICE.c_str(), O_RDWR | O_NOCTTY ); 
    if (this->fd < 0) 
    {
        perror(this->RSDEVICE.c_str());
        cout<<"Serial hasn't connected."<<endl;
        return this->fd; 
    }

    tcgetattr(this->fd,&this->oldtio);              /* save current serial port settings */
    bzero(&this->newtio, sizeof(this->newtio));     /* clear struct for new port settings */

//    BAUDRATE: устанавливает скорость передачи данных в bps.
//              Вы также могли бы использовать cfsetispeed и cfsetospeed.
//    CRTSCTS : аппаратное управление выводным потоком данных (используется
//              только если кабель обладает всеми необходимыми сигнальными
//              линиями. См. секцию 7 Serial-HOWTO)
//    CS8     : 8n1 (8 бит, без четности, 1 стоп-бит)
//    CSTOPB  : стоп-бит
//    CLOCAL  : локальное подключение, нет управления модемом
//    CREAD   : разрешает прием символов

    this->newtio.c_cflag = BAUDRATE | CS8 | CSTOPB | CREAD | CLOCAL;

//    IGNBRK  : игнорировать завершение
//    IGNPAR  : игнорировать байты с ошибками четности
//    ICRNL   : отобразить символ CR на NL (иначе ввод CR на другом
//              компьютере не будет завершать ввод)
//              иначе установить устройство как raw (нет обработки ввода)

    this->newtio.c_iflag = IGNBRK | ICRNL;

//    Raw вывод (нет обработки вывода).

    this->newtio.c_oflag = 0;

//    ICANON  : разрешить канонический ввод
//              заблокировать любое функционирование "эхо", и не посылать сигналы
//              к вызывающей программе

    this->newtio.c_lflag = 0;

//    инициализация всех управляющих символов
//    значения по умолчанию могут быть найдены в in /usr/include/termios.h,
//    и даются в комментариях, но мы не нуждаемся в них здесь

    this->newtio.c_cc[VTIME] = 10;  /* посимвольный таймер не используется */
//    Set minimum bytes to read
    this->newtio.c_cc[VMIN] = 0;    /* 1 means wait until at least 1 character is received */

//    теперь очищаем модемную линию и активируем наши установки порта
    tcsetattr(this->fd, TCSANOW, &this->newtio);
    
    return this->fd; 
}

bool Serial::Read(schunk_ezn64_driver::state &msg)
{
    unsigned char *comand = new unsigned char[21];
    unsigned char *buff = new unsigned char[4];
    bool pkgFind = false;
    int len, n; 
    
    n = 0;
    read(this->fd, &comand[0], 1);    
    
    while(!pkgFind && n < 20)
    {
        read(this->fd, &comand[1], 1);        
            
        msg.position = 0;
        msg.speed = 0;
        msg.current = 0;
        msg.status = "";
        msg.error = "";  
        
        if(comand[0] == 0x07 && comand[1] == 0x0C)
        {
            read(this->fd, &comand[2], 1);
            len = comand[2] + 5;            
            for(int i = 3; i < len; i++)
                read(this->fd, &comand[i], 1);
                
           unsigned short crc = Checksum_CRC16(comand, len - 2);
           if(comand[len - 2] == lo(crc) && comand[len - 1] == hi(crc))
           {
                pkgFind = true;
                
                if(comand[3] == 0x95)
                {
                    for(int i = 0; i < 4; i++)
                        buff[i] = comand[i + 4];
                    HexadToFloat(msg.position, buff);
                    
                    for(int i = 0; i < 4; i++)
                        buff[i] = comand[i + 8];
                    HexadToFloat(msg.speed, buff);
                    
                    for(int i = 0; i < 4; i++)
                        buff[i] = comand[i + 12];
                    HexadToFloat(msg.current, buff);                     

                    if (comand[16] & 0x01) msg.status = msg.status + "Referenced | "; 
                    if (comand[16] & 0x02) msg.status = msg.status + "Motion | ";
                    if (comand[16] & 0x04) msg.status = msg.status + "Prog seq | ";
                    if (comand[16] & 0x08) msg.status = msg.status + "Warning | ";
                    if (comand[16] & 0x10) msg.status = msg.status + "Error | ";
                    if (comand[16] & 0x20) msg.status = msg.status + "Brake | ";
                    if (comand[16] & 0x40) msg.status = msg.status + "Motion blocked | ";
                    if (comand[16] & 0x80) msg.status = msg.status + "Position reached  ";

                    if (comand[17] & 0x76) msg.error =       "ERROR cable break [0x76]";
                    else if (comand[17] == 0xDD) msg.error = "ERROR commutation [0xDD]";
                    else if (comand[17] == 0xD2) msg.error = "ERROR config memory [0xD2]";
                    else if (comand[17] == 0xDE) msg.error = "ERROR current [0xDE]";
                    else if (comand[17] == 0xD9) msg.error = "ERROR fast stop [0xD9]";
                    else if (comand[17] == 0xDC) msg.error = "ERROR fragmentation [0xDC]";
                    else if (comand[17] == 0xDF) msg.error = "ERROR I2T [0xDF]";
                    else if (comand[17] == 0xE0) msg.error = "ERROR initialize [0xE0]";
                    else if (comand[17] == 0xE1) msg.error = "ERROR internal [0xE1]";
                    else if (comand[17] == 0xD4) msg.error = "ERROR invalid phrase [0xD4]";
                    else if (comand[17] == 0x73) msg.error = "ERROR logic high [0x73]";
                    else if (comand[17] == 0x72) msg.error = "ERROR logic low [0x72]";
                    else if (comand[17] == 0xEC) msg.error = "ERROR math [0xEC]"; 
                    else if (comand[17] == 0x75) msg.error = "ERROR motor voltage high [0x75]"; 
                    else if (comand[17] == 0x74) msg.error = "ERROR motor voltage low [0x74]";
                    else if (comand[17] == 0x82) msg.error = "ERROR overshoot [0x82]";
                    else if (comand[17] == 0xD3) msg.error = "ERROR program memory [0xD3]"; 
                    else if (comand[17] == 0xEB) msg.error = "ERROR resolver check failed [0xEB]";
                    else if (comand[17] == 0xD8) msg.error = "ERROR service [0xD8]";
                    else if (comand[17] == 0xD6) msg.error = "ERROR soft high [0xD6]";
                    else if (comand[17] == 0xD5) msg.error = "ERROR soft low [0xD5]";
                    else if (comand[17] == 0x71) msg.error = "ERROR temp high [0x71]";
                    else if (comand[17] == 0x70) msg.error = "ERROR temp low [0x70]";
                    else if (comand[17] == 0xE4) msg.error = "ERROR too fast [0xE4]";
                    else if (comand[17] == 0xDA) msg.error = "ERROR tow [0xDA]";
                    else if (comand[17] == 0xD8) msg.error = "ERROR VPC3 [0xD8]";
                    else if (comand[17] == 0xC8) msg.error = "ERROR wrong ramp type [0xC8]";
                    else if (comand[17] == 0x06) msg.error = "ERROR referenced [0x06]";
                    else if (comand[17] == 0x83) msg.error = "ERROR hardware version [0x83]";
                    else if (len == 21 && comand[17] == 0x00 && comand[18] == 0x01)  
                                                 msg.error = "INFO boot [0x0001]";
                    else if (comand[17] == 0x19) msg.error = "INFO checksum [0x19]";
                    else if (comand[17] == 0x09) msg.error = "INFO communication error [0x09]";
                    else if (comand[17] == 0x05) msg.error = "INFO failed [0x05]";
                    else if (comand[17] == 0x1D) msg.error = "INFO message length [0x1D]";
                    else if (len == 21 && comand[17] == 0x00 && comand[18] == 0x08)  
                                                 msg.error = "INFO no error [0x0008]";
                    else if (comand[17] == 0x03) msg.error = "INFO no rights [0x03]";
                    else if (len == 21 && comand[17] == 0x00 && comand[18] == 0x07)  
                                                 msg.error = "INFO search sine vector [0x0007]";
                    else if (comand[17] == 0x10) msg.error = "INFO timeout [0x10]";
                    else if (comand[17] == 0x11) msg.error = "INFO unknown axis index [0x11]";
                    else if (comand[17] == 0x04) msg.error = "INFO unknown command [0x04]";
                    else if (comand[17] == 0x16) msg.error = "INFO wrong baudrate [0x16]";
                    else if (comand[17] == 0x1E) msg.error = "INFO wrong parameter [0x1E]";
                    else if (comand[17] == 0x00) msg.error = "INFO no error [0x00]";
                }
            }  
        }
        else
        {
            comand[0] = comand[1];
            n++;
        }
    }   
    return pkgFind;
}

int Serial::Write(strMsg msg) 
{
    int len = msg.len + 5;
    unsigned char *comand = new unsigned char[len];
    comand[0] = msg.state;
    comand[1] = msg.gripper;
    comand[2] = msg.len;
    comand[3] = msg.cmd;
    for(int i = 0; i < len - 6; i++)
        comand[i + 4] = msg.param[i];
     
    unsigned short crc = Checksum_CRC16(comand, len - 2);    
	comand[len - 2] = lo(crc);
	comand[len - 1] = hi(crc);

    int res = write(this->fd, comand, len);
    delete [] comand;
    
    return res; 
}

unsigned short Serial::Checksum_CRC16(unsigned char *pcBlock, unsigned short len)
{
    unsigned short crc = 0x0000;
    while (len--)
        crc = (crc >> 8) ^ crc16Tbl[(crc & 0xFF) ^ *pcBlock++];
    return crc;
}

void Serial::HexadToFloat(float &data, unsigned char *param)
{
    union 
    {
        float num;
        unsigned char chars[4];
    } val;
    
    for(int i = 0; i < 4; i++)
        val.chars[i] = param[i];
        
    data = val.num;
}

void Serial::Disconnect()
{
//  восстановление старых установок порта
    tcsetattr(this->fd,TCSANOW,&this->oldtio);
    close(this->fd);
    this->fd = -1;
    cout<<"Serial has disconnected."<<endl;
}
