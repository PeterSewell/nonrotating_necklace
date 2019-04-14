/* *****************************************************************
 radio_marshalling.h: marshal/unmarshal data between necklace state
 and radio packet
 ***************************************************************** */

/* Copyright 2019, Peter Sewell.  This is made available under  */
/* the BSD-2-Clause license in LICENSE                          */
// ****************************************************************


// buffer for marshalling radio data
radio_data r;


// marshal data to radio buffer (used in necklace.ino)
void marshal() {
  memcpy(r.araw, araw, sizeof(araw));
  memcpy(r.graw, graw, sizeof(graw));
  //  memcpy(r.mcalib, mcalib, sizeof(mcalib));
  memcpy(r.mraw, mraw, sizeof(mraw));
  r.north_angle = (uint8_t)( north_angle * (255.0f / twopi));
  r.gwz_integrated_magged = (uint8_t)( gwz_integrated_magged * (255.0f / twopi));
  // 500 degrees/s = approx 9 rad/s.  2000 degree/s (the max?) = approx 36 rad/s.   So scale  +- 40 rad/s to int16_t
  r.gwz = (int16_t)(gwz * (32700.0f / (40.0f * twopi)));
  for (i=0;i<3;i++)
    r.a_smoothed[i] = (int16_t)(a_smoothed[i] * (32700.0f / 2.0f));
  r.now = (uint16_t)(now & 0xFFFF);
  r.rate = (uint8_t)(rate);
  r.sample_count = (uint8_t)(sample_count & 0xFF);
}


// unmarshal data from radio buffer (used in necklace_receiver.ino)
#ifdef RECEIVER
uint16_t receiver_last_now_sample=0;
uint32_t receiver_now_offset=0;
void unmarshal() {
  memcpy(araw, r.araw, sizeof(araw));
  memcpy(graw, r.graw, sizeof(graw));
  //  memcpy(mcalib, r.mcalib, sizeof(mcalib));
  memcpy(mraw, r.mraw, sizeof(mraw));
  north_angle = ((float)r.north_angle) / (255.0f / twopi);
  gwz_integrated_magged = ((float)r.gwz_integrated_magged) / (255.0f / twopi);
  gwz = ((float)r.gwz) / (32700.0f / (40.0f * twopi));
  for (i=0;i<3;i++)
    a_smoothed[i] = ((float)r.a_smoothed[i]) / (32700.0f / 2.0f);
  if (r.now<receiver_last_now_sample) receiver_now_offset += 65536;
  receiver_last_now_sample=r.now;
  now=receiver_now_offset + (uint32_t)r.now;
  rate = (float)(r.rate);
  sample_count = (uint16_t)(r.sample_count);
}
#endif
