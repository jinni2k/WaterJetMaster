//Start of Modbus RTU
int CRC16;
int ElapsedTime;
uint8_t ClientFC, Bytes, Size;
int ClientStart, ServerStart, Quantity, Block, BlockName;
int Runs, Reads, Writes;
bool Active;
unsigned long PreviousActivityTime;

uint16_t MbReg[256];
uint8_t MbCoil[30];

uint8_t txBuf[264];
uint8_t rxBuf[264];
uint8_t Tquery[30];
uint8_t Rquery[30];

uint8_t MbDI[32];
uint8_t MbDO[32];
//End Of Moubus RTU

int i2c_Node;
int i2c_TX_length;
int i2c_RX_length;
int bytes;

unsigned int crc(uint8_t *buf, int start, int cnt) {
        int i, j;
        unsigned crc16, flag;
        crc16 = 0xFFFF;
        for (i = start; i < cnt; i++) {
                crc16 = crc16 ^ buf[i];
                for (j = 1; j <= 8; j++) {
                        flag = crc16 & 0x0001;
                        crc16 = crc16 >> 1;
                        if (flag) crc16 = crc16 ^ 0xA001;
                }
        }
        return (lowByte(crc16) * 0x100 + highByte(crc16));
}

void MBStransmit(byte port, int Mod_length) {
        wdt_reset();
        switch (port) {
        case 0: digitalWrite(D7, HIGH); Serial.write(txBuf, Mod_length); Serial.flush(); delay(1); break;
        case 1: digitalWrite(D7, HIGH); Serial1.write(txBuf, Mod_length); Serial1.flush(); delay(1); break;
                //case 0: digitalWrite(D4, HIGH); Serial.write(txBuf, Mod_length); Serial.flush(); delay(1); digitalWrite(D4, LOW); break;
                //case 2: digitalWrite(D4, HIGH); Serial2.write(txBuf, Mod_length); Serial2.flush(); delay(1); digitalWrite(D4, LOW); break;
                //    case 2:Serial2.write(txBuf,Mod_length);Serial2.flush();break;
                //    case 3:Serial3.write(txBuf,Mod_length);Serial3.flush();break;
                //    case 4:i2c_TX_Mod_length=Mod_length;break;
        }
}

void MBSerror(byte port, byte slave) {
        txBuf[0] = slave;
        txBuf[1] = txBuf[1] + 0x80;
        txBuf[2] = 0x02;
        CRC16 = crc(txBuf, 0, 3);
        txBuf[3] = highByte(CRC16);
        txBuf[4] = lowByte(CRC16);
        MBStransmit(port, 0x05);
}

void MBSlave(byte port, byte slave) {
        int start, Mod_length;
        int bytes, i, j;
        bool Coil[1024];
        //digitalWrite(D4, LOW);   //TXEN
        wdt_reset();
        switch (port) {
        case 0:
                for (i = 0; 0 < Serial.available(); i++) {
                        txBuf[i] = Serial.read();
                        delay(1);
                } break;
        case 1:
                for (i = 0; 0 < Serial1.available(); i++) {
                        txBuf[i] = Serial1.read();
                        delay(1);
                } break;

                //    case 2:for(i=0;0<Serial2.available();i++)txBuf[i]= Serial2.read();break;
                //    case 3:for(i=0;0<Serial3.available();i++)txBuf[i]= Serial3.read();break;
                //    case 4:for(i=0;1<Wire.available();i++)txBuf[i]= Wire.read();int x = Wire.read();break;

        }
        if (txBuf[0] == slave && i >= 0x08) {
                // Serial.println("03 Fuction.");
                if ((txBuf[1] == 0x03) || (txBuf[1] == 0x04)) { //Multiple Registers reading
                        CRC16 = crc(txBuf, 0, 6);
                        start = (int)txBuf[2] * 256 + (int)txBuf[3];
                        Mod_length = (int)txBuf[4] * 256 + (int)txBuf[5];
                        if (txBuf[6] == highByte(CRC16) && txBuf[7] == lowByte(CRC16)) {
                                txBuf[2] = (byte)Mod_length * 2;
                                for (i = 0; i < Mod_length; i++) {
                                        txBuf[3 + i * 2] = highByte(MbReg[start + i]);
                                        txBuf[3 + i * 2 + 1] = lowByte(MbReg[start + i]);
                                }
                                CRC16 = crc(txBuf, 0, Mod_length * 2 + 3);
                                txBuf[Mod_length * 2 + 3] = highByte(CRC16);
                                txBuf[Mod_length * 2 + 4] = lowByte(CRC16);
                                bytes = (int)Mod_length * 2 + 5;
                                MBStransmit(port, bytes);
                        }
                        else if (port < 4) MBSerror(port, slave);
                }
                else if (txBuf[1] == 0x17) { //Multiple Registers reading and Writing
                        start = (int)txBuf[6] * 256 + (int)txBuf[7];//Write
                        Mod_length = (int)txBuf[8] * 256 + (int)txBuf[9];
                        bytes = (int)txBuf[10];
                        CRC16 = crc(txBuf, 0, 11 + bytes);
                        if (txBuf[11 + bytes] == highByte(CRC16) && txBuf[12 + bytes] == lowByte(CRC16)) {
                                for (i = 0; i < Mod_length; i++) MbReg[start + i] = (int)txBuf[11 + i * 2] * 256 + (int)txBuf[12 + i * 2];
                                start = (int)txBuf[2] * 256 + (int)txBuf[3];//Read
                                Mod_length = (int)txBuf[4] * 256 + (int)txBuf[5];
                                txBuf[2] = (byte)Mod_length * 2;
                                for (i = 0; i < Mod_length; i++) {
                                        txBuf[3 + i * 2] = highByte(MbReg[start + i]);
                                        txBuf[3 + i * 2 + 1] = lowByte(MbReg[start + i]);
                                }
                                CRC16 = crc(txBuf, 0, Mod_length * 2 + 3);
                                txBuf[Mod_length * 2 + 3] = highByte(CRC16);
                                txBuf[Mod_length * 2 + 4] = lowByte(CRC16);
                                bytes = (int)Mod_length * 2 + 5;
                                MBStransmit(port, bytes);
                        }
                        else if (port < 4) MBSerror(port, slave);
                }
                else if (txBuf[1] == 0x10) { //Multiple Registers writing
                        start = (int)txBuf[2] * 256 + (int)txBuf[3];
                        Mod_length = (int)txBuf[4] * 256 + (int)txBuf[5];
                        bytes = (int)txBuf[6];
                        CRC16 = crc(txBuf, 0, 7 + bytes);
                        if (txBuf[7 + bytes] == highByte(CRC16) && txBuf[8 + bytes] == lowByte(CRC16)) {
                                for (i = 0; i < Mod_length; i++) MbReg[start + i] = (int)txBuf[7 + i * 2] * 256 + (int)txBuf[8 + i * 2];
                                CRC16 = crc(txBuf, 0, 6);
                                txBuf[6] = highByte(CRC16);
                                txBuf[7] = lowByte(CRC16);
                                bytes = 16;
                                MBStransmit(port, bytes);
                        }
                        else if (port < 4) MBSerror(port, slave);
                }
                else if ((txBuf[1] == 0x01) || (txBuf[1] == 0x02)) { //Multiple Digital reading
                        CRC16 = crc(txBuf, 0, 6);
                        start = (int)txBuf[2] * 256 + (int)txBuf[3];
                        Mod_length = (int)txBuf[4] * 256 + (int)txBuf[5];
                        if (txBuf[6] == highByte(CRC16) && txBuf[7] == lowByte(CRC16)) {
                                if (Mod_length % 8 == 0) bytes = (int)Mod_length / 8;
                                else bytes = (int)Mod_length / 8 + 1;
                                txBuf[2] = (byte)bytes;
                                for (i = start, j = 0; i < start + Mod_length; i++, j++) Coil[j] = MbCoil[i];
                                for (i = 0; i < bytes; i++) for (j = 0; j < 8; j++) bitWrite(txBuf[3 + i], j, Coil[i * 8 + j]);
                                CRC16 = crc(txBuf, 0, bytes + 3);
                                txBuf[bytes + 3] = highByte(CRC16);
                                txBuf[bytes + 4] = lowByte(CRC16);
                                bytes = bytes + 5;
                                MBStransmit(port, bytes);
                        }
                        else if (port < 4) MBSerror(port, slave);
                }
                else if (txBuf[1] == 0x06) { //Single Register writing
                        CRC16 = crc(txBuf, 0, 6);
                        start = (int)txBuf[2] * 256 + (int)txBuf[3];
                        if (txBuf[6] == highByte(CRC16) && txBuf[7] == lowByte(CRC16)) {
                                MbReg[start] = (int)txBuf[4] * 256 + (int)txBuf[5];
                                bytes = 8;
                                MBStransmit(port, bytes);
                        }
                        else if (port < 4) MBSerror(port, slave);
                }
                else if (txBuf[1] == 0x05) { //Single Digital writing
                        CRC16 = crc(txBuf, 0, 6);
                        start = (int)txBuf[2] * 256 + (int)txBuf[3];
                        if (txBuf[6] == highByte(CRC16) && txBuf[7] == lowByte(CRC16)) {
                                if (txBuf[4] == 0xFF) MbCoil[start] = 1;
                                else MbCoil[start] = 0;
                                bytes = 8;
                                MBStransmit(port, bytes);
                        }
                        else if (port < 4) MBSerror(port, slave);
                }
                else if (txBuf[1] == 0x0F) { //Multiple Digital writing
                        start = (int)txBuf[2] * 256 + (int)txBuf[3];
                        Mod_length = (int)txBuf[4] * 256 + (int)txBuf[5];
                        bytes = 7 + txBuf[6];
                        CRC16 = crc(txBuf, 0, bytes);
                        if (txBuf[bytes] == highByte(CRC16) && txBuf[bytes + 1] == lowByte(CRC16)) {
                                for (i = 0; i < Mod_length; i++) Coil[i] = txBuf[7 + i / 8] & (0x01 << i % 8);
                                for (i = 0; i < Mod_length; i++) MbCoil[start + i] = Coil[i];
                                CRC16 = crc(txBuf, 0, 6);
                                txBuf[6] = highByte(CRC16);
                                txBuf[7] = lowByte(CRC16);
                                bytes = 8;
                                MBStransmit(port, bytes);
                        }
                        else if (port < 4) MBSerror(port, slave);
                }
        }
}


////////////////Modbus Master
void MBMtransmit(byte port, int TX_length) {
        switch (port) {
        case 0: digitalWrite(D7, HIGH); Serial.write(txBuf, TX_length); Serial.flush(); delay(1); break;
        case 1: digitalWrite(D7, HIGH); Serial1.write(txBuf, TX_length); Serial1.flush(); delay(1); break;
                /*
                   case 2:Serial2.write(txBuf,TX_length);break;
                   case 3:Serial3.write(txBuf,TX_length);break;
                   //i2c Transmission and Request port 4
                   case 4:Wire.beginTransmission(i2c_Node);
                       Wire.write(txBuf,TX_length);Wire.write(" ");
                       Wire.endTransmission();

                       break;
                 */
        }
        wdt_reset();
}

void MBMresponse(byte port, byte slave, int address, int length, int Mb_Add) {
        //if (port < 4)delay(50); delay(2 + bytes / 16); wdt_reset();
        int i = 0;
        switch (port) {
        case 0:
                //while(Serial.available()<5){wdt_reset();}delay(10);
                while (Serial.available() > 0) {
                        int BufSize = Serial.available();
                        //delay(2);
                        if (BufSize == Serial.available()) {
                                for (i = 0; 0 < Serial.available(); i++) rxBuf[i] = Serial.read();
                        }
                }
                break;
        case 1:
                //while(Serial1.available()<5){wdt_reset();}delay(10);
                while (Serial1.available() > 0) {
                        int BufSize = Serial1.available();
                        //delay(2);
                        if (BufSize == Serial1.available()) {
                                for (i = 0; 0 < Serial1.available(); i++) rxBuf[i] = Serial1.read();
                        }
                }
                break;
                /*
                   case 2:
                   //while(Serial2.available()<5){wdt_reset();}delay(10);
                   while(Serial2.available()>0){
                    int BufSize=Serial2.available();
                    delay(2);
                    if(BufSize==Serial2.available()){for(int i=0;0<Serial2.available();i++)rxBuf[i]= Serial2.read();}
                   }
                   break;
                   case 3:
                   //while(Serial3.available()<5){wdt_reset();}delay(10);
                   while(Serial3.available()>0){
                    int BufSize=Serial3.available();
                    delay(2);
                    if(BufSize==Serial3.available()){for(int i=0;0<Serial3.available();i++)rxBuf[i]= Serial3.read();}
                   }
                   break;
                   case 4:
                   Wire.requestFrom(i2c_Node,i2c_RX_length);
                   for(int i=0;i<i2c_RX_length;i++)if(Wire.available())rxBuf[i]=Wire.read();
                   //i2c_Debug();
                   break;
                 */
        }

        if (rxBuf[0] == slave && i >= 0x06) {
                if (rxBuf[1] == 0x03 && rxBuf[2] == length * 2) { //Multiple Holding Registers reading
                        CRC16 = crc(rxBuf, 0, 3 + rxBuf[2]);
                        if (rxBuf[3 + rxBuf[2]] == highByte(CRC16) && rxBuf[4 + rxBuf[2]] == lowByte(CRC16)) {
                                //if(port=4)i2c_Indicator();
                                for (int i = 0; i < length; i++) {
                                        MbReg[Mb_Add + i] = word(rxBuf[i * 2 + 3], rxBuf[i * 2 + 4]);
                                }
                        }
                }
                else if (rxBuf[1] == 0x04 && rxBuf[2] == length * 2) { //Multiple Input Registers reading
                        CRC16 = crc(rxBuf, 0, 3 + rxBuf[2]);
                        if (rxBuf[3 + rxBuf[2]] == highByte(CRC16) && rxBuf[4 + rxBuf[2]] == lowByte(CRC16)) {
                                //if(port=4)i2c_Indicator();
                                for (int i = 0; i < length; i++) {
                                        MbReg[Mb_Add + i] = word(rxBuf[i * 2 + 3], rxBuf[i * 2 + 4]);
                                }
                        }
                }
                else if (rxBuf[1] == 0x17 && rxBuf[2] == length * 2) { //Multiple Holding Registers reading and Writing
                        CRC16 = crc(rxBuf, 0, 3 + rxBuf[2]);
                        if (rxBuf[3 + rxBuf[2]] == highByte(CRC16) && rxBuf[4 + rxBuf[2]] == lowByte(CRC16)) {
                                //if(port=4)i2c_Indicator();
                                for (int i = 0; i < length; i++) {
                                        MbReg[Mb_Add + i] = word(rxBuf[i * 2 + 3], rxBuf[i * 2 + 4]);
                                }
                        }
                }
                else if ((rxBuf[1] == 0x01 || rxBuf[1] == 0x02)) { //Multiple Digital reading
                        CRC16 = crc(rxBuf, 0, 3 + rxBuf[2]);
                        if (rxBuf[3 + rxBuf[2]] == highByte(CRC16) && rxBuf[4 + rxBuf[2]] == lowByte(CRC16)) {
                                //if(port=4)i2c_Indicator();
                                int counts;
                                if (length % 8 == 0) counts = length / 8; else counts = length / 8 + 1;
                                for (int i = 0; i < counts; i++) for (int j = 0; j < 8; j++) if (i * 8 + j < length) MbCoil[Mb_Add + i * 8 + j] = bitRead(rxBuf[3 + i], j);
                        }
                }
                else if (rxBuf[1] == 0x06 || rxBuf[1] == 0x10) {
                        CRC16 = crc(rxBuf, 0, 6);
                        if (rxBuf[7] == highByte(CRC16) && rxBuf[8] == lowByte(CRC16)) {
                                //if(port=4)i2c_Indicator();
                                //CommOk
                        }
                }
                else if (rxBuf[1] == 0x05 || rxBuf[1] == 0x0F) {
                        CRC16 = crc(rxBuf, 0, 6);
                        if (rxBuf[7] == highByte(CRC16) && rxBuf[8] == lowByte(CRC16)) {
                                //if(port=4)i2c_Indicator();
                                //CommOk
                        }
                }
                for (int i = 0; i < sizeof(rxBuf); i++) rxBuf[i] = 0;
        }
}
void MBMaster(byte port, byte slave, byte func, int address, int length, int Mb_Add) {
        if (func == 1) {
                txBuf[0] = slave;
                txBuf[1] = 0x01;
                txBuf[2] = highByte(address);
                txBuf[3] = lowByte(address);
                txBuf[4] = highByte(length);
                txBuf[5] = lowByte(length);
                CRC16 = crc(txBuf, 0, 6);
                txBuf[6] = highByte(CRC16);
                txBuf[7] = lowByte(CRC16);
                i2c_RX_length = 5 + length / 8; if (length % 8 > 0) i2c_RX_length++;
                bytes = 8;
                MBMtransmit(port, bytes);
                wdt_reset();
                MBMresponse(port, slave, address, length, Mb_Add);
        }
        else if (func == 2) {
                txBuf[0] = slave;
                txBuf[1] = 0x02;
                txBuf[2] = highByte(address);
                txBuf[3] = lowByte(address);
                txBuf[4] = highByte(length);
                txBuf[5] = lowByte(length);
                CRC16 = crc(txBuf, 0, 6);
                txBuf[6] = highByte(CRC16);
                txBuf[7] = lowByte(CRC16);
                i2c_RX_length = 5 + length / 8; if (length % 8 > 0) i2c_RX_length++;
                bytes = 8;
                MBMtransmit(port, bytes);
                wdt_reset();
                MBMresponse(port, slave, address, length, Mb_Add);
        }
        else if (func == 3) {
                txBuf[0] = slave;
                txBuf[1] = 0x03;
                txBuf[2] = highByte(address);
                txBuf[3] = lowByte(address);
                txBuf[4] = highByte(length);
                txBuf[5] = lowByte(length);
                CRC16 = crc(txBuf, 0, 6);
                txBuf[6] = highByte(CRC16);
                txBuf[7] = lowByte(CRC16);
                i2c_RX_length = 5 + length * 2;
                bytes = 8;
                MBMtransmit(port, bytes);
                wdt_reset();
                MBMresponse(port, slave, address, length, Mb_Add);
        }
        else if (func == 4) {
                txBuf[0] = slave;
                txBuf[1] = 0x04;
                txBuf[2] = highByte(address);
                txBuf[3] = lowByte(address);
                txBuf[4] = highByte(length);
                txBuf[5] = lowByte(length);
                CRC16 = crc(txBuf, 0, 6);
                txBuf[6] = highByte(CRC16);
                txBuf[7] = lowByte(CRC16);
                i2c_RX_length = 5 + length * 2;
                bytes = 8;
                MBMtransmit(port, bytes);
                wdt_reset();
                MBMresponse(port, slave, address, length, Mb_Add);
        }
        else if (func == 5) {
                txBuf[0] = slave;
                txBuf[1] = 0x05;
                txBuf[2] = highByte(address);
                txBuf[3] = lowByte(address);
                if (MbCoil[Mb_Add]) txBuf[4] = 0xFF; else txBuf[4] = 0x00;
                txBuf[5] = 0x00;
                CRC16 = crc(txBuf, 0, 6);
                txBuf[6] = highByte(CRC16);
                txBuf[7] = lowByte(CRC16);
                i2c_RX_length = 8;
                bytes = 8;
                MBMtransmit(port, bytes);
                wdt_reset();
                MBMresponse(port, slave, address, length, Mb_Add);
        }
        else if (func == 15) {
                txBuf[0] = slave;
                txBuf[1] = 0x0F;
                txBuf[2] = highByte(address);
                txBuf[3] = lowByte(address);
                txBuf[4] = highByte(length);
                txBuf[5] = lowByte(length);
                if (length % 8 == 0) txBuf[6] = (byte)(length / 8); else txBuf[6] = (byte)(length / 8 + 1);
                for (int i = 0; i < txBuf[6]; i++) {
                        txBuf[i + 7] = 0;
                        if (length > 0) {
                                txBuf[i + 7] = txBuf[i + 7] * 2 + MbCoil[Mb_Add + i * 8 + 7];
                                length--;
                        }
                        if (length > 0) {
                                txBuf[i + 7] = txBuf[i + 7] * 2 + MbCoil[Mb_Add + i * 8 + 6];
                                length--;
                        }
                        if (length > 0) {
                                txBuf[i + 7] = txBuf[i + 7] * 2 + MbCoil[Mb_Add + i * 8 + 5];
                                length--;
                        }
                        if (length > 0) {
                                txBuf[i + 7] = txBuf[i + 7] * 2 + MbCoil[Mb_Add + i * 8 + 4];
                                length--;
                        }
                        if (length > 0) {
                                txBuf[i + 7] = txBuf[i + 7] * 2 + MbCoil[Mb_Add + i * 8 + 3];
                                length--;
                        }
                        if (length > 0) {
                                txBuf[i + 7] = txBuf[i + 7] * 2 + MbCoil[Mb_Add + i * 8 + 2];
                                length--;
                        }
                        if (length > 0) {
                                txBuf[i + 7] = txBuf[i + 7] * 2 + MbCoil[Mb_Add + i * 8 + 1];
                                length--;
                        }
                        if (length > 0) {
                                txBuf[i + 7] = txBuf[i + 7] * 2 + MbCoil[Mb_Add + i * 8 + 0];
                                length--;
                        }
                }
                CRC16 = crc(txBuf, 0, 7 + txBuf[6]);
                txBuf[7 + txBuf[6]] = highByte(CRC16);
                txBuf[8 + txBuf[6]] = lowByte(CRC16);
                i2c_RX_length = 8;
                bytes = 9 + txBuf[6];
                MBMtransmit(port, bytes);
                wdt_reset();
                MBMresponse(port, slave, address, length, Mb_Add);
        }
        else if (func == 6) {
                txBuf[0] = slave;
                txBuf[1] = 0x06;
                txBuf[2] = highByte(address);
                txBuf[3] = lowByte(address);
                txBuf[4] = highByte(MbReg[Mb_Add]);
                txBuf[5] = lowByte(MbReg[Mb_Add]);
                CRC16 = crc(txBuf, 0, 6);
                txBuf[6] = highByte(CRC16);
                txBuf[7] = lowByte(CRC16);
                i2c_RX_length = 8;
                bytes = 8;
                MBMtransmit(port, bytes);
                wdt_reset();
                MBMresponse(port, slave, address, length, Mb_Add);
        }
        else if (func == 16) {
                txBuf[0] = slave;
                txBuf[1] = 0x10;
                txBuf[2] = highByte(address);
                txBuf[3] = lowByte(address);
                txBuf[4] = highByte(length);
                txBuf[5] = lowByte(length);
                txBuf[6] = (byte)(length * 2);
                for (int i = 0; i < length; i++) {
                        txBuf[i * 2 + 7] = highByte(MbReg[Mb_Add + i]);
                        txBuf[i * 2 + 8] = lowByte(MbReg[Mb_Add + i]);
                }
                CRC16 = crc(txBuf, 0, 7 + length * 2);
                txBuf[7 + length * 2] = highByte(CRC16);
                txBuf[8 + length * 2] = lowByte(CRC16);
                i2c_RX_length = 8;
                bytes = 9 + length * 2;
                MBMtransmit(port, bytes);
                wdt_reset();
                MBMresponse(port, slave, address, length, Mb_Add);
        }
}
void MBMaster(byte port, byte slave, byte func, int Read_address, int Read_length, int Read_Mb_Add, int Write_address, int Write_length, int Write_Mb_Add) {
        if (func == 23) {
                txBuf[0] = slave;
                txBuf[1] = 0x17;
                txBuf[2] = highByte(Read_address);
                txBuf[3] = lowByte(Read_address);
                txBuf[4] = highByte(Read_length);
                txBuf[5] = lowByte(Read_length);
                txBuf[6] = highByte(Write_address);
                txBuf[7] = lowByte(Write_address);
                txBuf[8] = highByte(Write_length);
                txBuf[9] = lowByte(Write_length);
                txBuf[10] = (byte)(Write_length * 2);
                for (int i = 0; i < Write_length; i++) {
                        txBuf[i * 2 + 11] = highByte(MbReg[Write_Mb_Add + i]);
                        txBuf[i * 2 + 12] = lowByte(MbReg[Write_Mb_Add + i]);
                }
                CRC16 = crc(txBuf, 0, 11 + Write_length * 2);
                txBuf[11 + Write_length * 2] = highByte(CRC16);
                txBuf[12 + Write_length * 2] = lowByte(CRC16);
                i2c_RX_length = 5 + Write_length * 2;
                bytes = 13 + Write_length * 2;
                MBMtransmit(port, bytes);
                wdt_reset();
                MBMresponse(port, slave, Read_address, Read_length, Read_Mb_Add);
        }
}
