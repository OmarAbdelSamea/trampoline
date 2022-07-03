#include "ecdsa.h"
#include "rand.h"

uint8_t sig[300], 
priv_key[100] = "519b423d715f8b581f4fa8ee59f4771a5b44c8130b4e3eacca54a56dda72b464", 
msg[300] = "5905238877c77421f73e43ee3da6f2d9e2ccad5fc942dcec0cbd25482935faaf416983fe165b1a045ee2bcd2e6dca3bdf46c4310a7461f9a37960ca672d3feb5473e253605fb1ddfd28065b53cb5858a8ad28175bf9bd386a5e471ea7a65c17cc934a9d791e91491eb3754d03799790fe2d308d16146d5c9b0d0debd97d79ce8\0";
uint32_t sig_len, i, msg_len = 256;
int cnt = 0;

int main()
{
  while(1) {
    ledToggle(GREEN);
    ecdsa_sign(priv_key, msg, msg_len, sig, &sig_len);
    ledToggle(RED);
    ledToggle(GREEN);
  }
}