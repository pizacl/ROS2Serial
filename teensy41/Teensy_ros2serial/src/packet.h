#pragma once

#include <stdint.h>

#define BUFFER_RX_SIZE 32768
#define BUFFER_TX_SIZE 32768

uint8_t _packet_tx_decode[BUFFER_TX_SIZE];
uint8_t _packet_tx_encode[BUFFER_TX_SIZE];

uint8_t _packet_rx_decode[BUFFER_RX_SIZE];
uint8_t _packet_rx_encode[BUFFER_RX_SIZE];

int _packet_rx_len = 0;
int _packet_tx_len = 0;

// 必要な関数
void packet_recv_cb(const int id, const int len, const uint8_t* data);

size_t _packet_cobsEncode(const uint8_t *data, size_t length, uint8_t *buffer){
    // assert(data && buffer);

    uint8_t *encode = buffer; // Encoded byte pointer
    uint8_t *codep = encode++; // Output code pointer
    uint8_t code = 1; // Code value

    for (const uint8_t *byte = (const uint8_t *)data; length--; ++byte)
    {
        if (*byte) // Byte not zero, write it
            *encode++ = *byte, ++code;

        if (!*byte || code == 0xff) // Input is zero or block completed, restart
        {
            *codep = code, code = 1, codep = encode;
            if (!*byte || length)
                ++encode;
        }
    }
    *codep = code; // Write final code value

    return (size_t)(encode - buffer);
}

size_t _packet_cobsDecode(const uint8_t *buffer, size_t length, uint8_t *data){
    // assert(buffer && data);

    const uint8_t *byte = buffer; // Encoded input byte pointer
    uint8_t *decode = (uint8_t *)data; // Decoded output byte pointer

    for (uint8_t code = 0xff, block = 0; byte < buffer + length; --block)
    {
        if (block) // Decode block byte
            *decode++ = *byte++;
        else
        {
            block = *byte++;             // Fetch the next block length
            if (block && (code != 0xff)) // Encoded zero, write it unless it's delimiter.
                *decode++ = 0;
            code = block;
            if (!code) // Delimiter code found
                break;
        }
    }

    return (size_t)(decode - (uint8_t *)data);
}

void packet_send(){
  Serial.write(_packet_tx_encode, _packet_tx_len);
  _packet_tx_len = 0;
}

bool packet_setUint8(const uint8_t id, const int u8_len, const uint8_t *data){
  if ((_packet_tx_len + u8_len + 3) >= BUFFER_TX_SIZE)
      return false;
  _packet_tx_decode[0] = id;
  _packet_tx_decode[1] = u8_len;
  memcpy(_packet_tx_decode+2, data, u8_len);
  int len = _packet_cobsEncode(_packet_tx_decode, (u8_len + 2), _packet_tx_encode+_packet_tx_len);
  _packet_tx_len += len;
  _packet_tx_encode[_packet_tx_len] = 0;
  _packet_tx_len++;
  return true;
}

// PCに接続
void packet_begin(){
    Serial.begin(9600);
}

// PCから受信したデータを解析
void packet_update(){
  while(Serial.available()){
    if(_packet_rx_len == (BUFFER_RX_SIZE-1)){
      _packet_rx_len = 0;
    }
    uint8_t data = Serial.read();
    if(data == 0){
      int size = _packet_cobsDecode(_packet_rx_encode, _packet_rx_len, _packet_rx_decode);

      if(_packet_rx_len != (size + 1)){
        _packet_rx_len = 0;
        continue;    
      }
      int id_can = _packet_rx_decode[0];
      int len_uint8 = _packet_rx_decode[1];
      // 0x00, id, len, data
      if(2 + len_uint8 != size){
        _packet_rx_len = 0;
        continue; 
      }

      packet_recv_cb(id_can, len_uint8, &_packet_rx_decode[2]);
      _packet_rx_len = 0;
    }else{
      _packet_rx_encode[_packet_rx_len++] = data;
    }
  }
}