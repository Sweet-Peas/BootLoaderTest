volatile uint32_t msTicks;
#define RECV_BUFFER_SIZE 512
unsigned char data_buf[RECV_BUFFER_SIZE];

#define FLASH_SECTOR_SIZE 0x400

#define CMD_BOOT_VERSION   0x00
#define CMD_CHIP_ID        0x02
#define CMD_WRITE_FLASH    0x31
#define CMD_READ_FLASH     0x11
#define CMD_ERASE_FLASH    0x43
#define CMD_UNKNOW         0xFF
#define RES_ACK            0x79
#define RES_NACK           0x1F
#define RES_UNKNOW         0xEE

#define USER_CODE_ADDRESS  0x01000
#define TOP_CODE_ADDRESS   0x01FFF

// System pins
#define UPDATE_PIN         4

#define flash_write_data(ptr,addr,len)
#define flash_erase_page(addr)

/* Enable or disable debug logs */
#define OFF                0
#define ON                 1
#define DEBUG              OFF

#if DEBUG == ON
#define DBG(x)             x
#else
#define DBG(x)
#endif

#define Host               Serial
#define Debug              Serial1

byte ledState = 0;

void setup() {
  pinMode(UPDATE_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Serial connects to the host
  Host.begin(115200);
  // and Serial1 connects to the debug terminal
  DBG(Debug.begin(115200));
  
  // Wait for the serial ports to connect
  DBG(while(!Debug));
  DBG(Debug.println(F("\r\nDebug channel acquired !")));
  
  // Wait for 0x7F sync word
  while (!Host.available());

  
  DBG(Debug.println(F("\r\nDebug terminal !")));
  DBG(Debug.print(F("> ")));
}

void toggleLed(byte led)
{
  if (!ledState) {
    ledState = 1;
    digitalWrite(led, HIGH);
  } else {
    ledState = 0;
    digitalWrite(led, LOW);
  }
}

void uart_send_data(unsigned char *buf, unsigned short len)
{
  while (len--) {
    Host.write(*buf);
    buf++;
  }
}

unsigned short uart_get_data(unsigned char *buf, unsigned short len)
{
  while (len) {
    // Wait for character to arrive
    while (!Host.available());
    *buf = Host.read();
    buf++;
    len--;
  }
  return len;
}

// Returns a non-zero value if an update is needed
unsigned int check_update_requirement(void)
{
   // PB2.8 high require update, low -- run application.
   if (digitalRead(UPDATE_PIN)) {
      return 1;
   } else {
      return 0;
   }
}

void bl_send_ack(unsigned char ack)
{
   unsigned char res = ack;
   uart_send_data(&res, 1);
}

int bl_get_cmd(void)
{
  unsigned short rcvd;
  unsigned char *ptr = data_buf;

  // Check for both system start sync and loader commands
  rcvd = uart_get_data(ptr, 1);
  if (!rcvd && ptr[0] == 0x7f) {
      Host.write(RES_ACK);
      return 0;
  }
  
  // Get next character in stream from host
  rcvd = uart_get_data(ptr+1, 1);
  if (rcvd == 0) {
    if (ptr[0] != (ptr[1] ^ 0xFF)) {
       bl_send_ack(RES_NACK);
       return -2;
    } else {
       bl_send_ack(RES_ACK);
       return 0;
    }
  } 
  return -1;
}

void bl_write_flash()
{
   int i;
   unsigned int addr, len;
   unsigned char crc;
   unsigned char *ptr = data_buf;
   toggleLed(LED_BUILTIN);
   // get address
   if (uart_get_data(ptr, 5) == 0) {
      crc = ptr[0] ^ ptr[1] ^ ptr[2] ^ ptr[3];
      if (crc == ptr[4]) {
         addr = ptr[3] | (ptr[2] << 8) | (ptr[1] << 16) | (ptr[0] << 24);
      }
   }
   // send ack
   bl_send_ack(RES_ACK);
   // get data len
   uart_get_data(ptr, 1);
   len = ptr[0] + 1;
   // get data and crc
   if (uart_get_data(ptr, len + 1) == 0) {
      crc = 0xFF;
      for (i = 0; i < len; i++) {
         crc = crc ^ ptr[i];
      }
   }
   //write flash
   if (crc == ptr[len]) {
      if ((addr >= USER_CODE_ADDRESS) && addr < TOP_CODE_ADDRESS) {
         flash_write_data(ptr, addr, len);
      }
   }
   // send ack
   bl_send_ack(RES_ACK);
}

void bl_read_flash()
{
   int i;
   unsigned int addr, len = 0;
   unsigned char crc;
   unsigned char *ptr = data_buf;
   // get address
   if (uart_get_data(ptr, 5) == 0) {
      crc = ptr[0] ^ ptr[1] ^ ptr[2] ^ ptr[3];
      if (crc == ptr[4]) {
         addr = ptr[3] | (ptr[2] << 8) | (ptr[1] << 16) | (ptr[0] << 24);
      }
   }
   // send ack
   bl_send_ack(RES_ACK);
   // get data len and crc
   if(uart_get_data(ptr, 2) == 0) {
      if((ptr[0] ^ 0xFF) == ptr[1]) {
         len = ptr[0] + 1;
      }
   }
   // send ack
   bl_send_ack(RES_ACK);
   // send data
   for(i =0; i < len; i++) {
      ptr[i]= *(unsigned char *)(addr++);
   }
   uart_send_data(ptr,len);
}

void bl_erase_flash()
{
   int i;
   unsigned int addr, len;
   unsigned char crc;
   unsigned char *ptr = data_buf;

   // get len
   if (uart_get_data(ptr, 1) == 0) {
      len = (ptr[0] + 1) & 0xFF; // maximum sector number is 256
   }
   // get sector sequence and crc
   if (uart_get_data(ptr, len + 1) == 0) {
      crc = 0xFF;
      for (i = 0; i < len; i++)
      {
         crc = crc ^ ptr[i];
      }
   }
   // erase flash sectors
   if (crc == ptr[len]) {
      for (i = 0; i < len; i++) {
         addr = ptr[i] * FLASH_SECTOR_SIZE;
         if ((addr >= USER_CODE_ADDRESS) && addr < TOP_CODE_ADDRESS) {
            flash_erase_page(addr);
         }
      }
   }
   // send ack
   bl_send_ack(RES_ACK);
}

void bl_boot_version()
{
   unsigned char buf[10];

   buf[0] = 5;  // Number of available commands
   buf[1] = 1;  // Version indicator
   buf[2] = 0x00;
   buf[3] = 0x02;
   buf[4] = 0x31;
   buf[5] = 0x11;
   buf[6] = 0x43;
   uart_send_data(buf, 7);
   
   bl_send_ack(RES_ACK);   
}

void bl_chip_id()
{
   unsigned char buf[10];

   buf[0] = 1;  // Number of bytes in id + 1
   buf[1] = 0x08;  // Version indicator
   buf[2] = 0x01;
   uart_send_data(buf, 3);
   
   bl_send_ack(RES_ACK);   
}

void loop() 
{
  if (bl_get_cmd() == 0) {
    switch (data_buf[0]) {
    case CMD_BOOT_VERSION:
      bl_boot_version();
      break;
    case CMD_CHIP_ID:
      bl_chip_id();
      break;
    case CMD_WRITE_FLASH:
      bl_write_flash();
      break;
    case CMD_READ_FLASH:
      bl_read_flash();
      break;
    case CMD_ERASE_FLASH:
      bl_erase_flash();
      break;
      
    default:
      break;
    }
  }

}

