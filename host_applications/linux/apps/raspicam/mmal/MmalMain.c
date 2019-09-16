#include "MmalEncoder.h"
#include "MmalEncoderInterface.h"

FILE *fp;

int mWidth = 640;
int mHeight = 480;
int initKbps = 1000;
int initFps = 30;

void callback(char *encoded_data, int length, unsigned long long int pts, unsigned long long int captured_time)
{
  printf("Main: len=%d, pts=%lld, cap=%lld\n", length, pts, captured_time);
  if (fp != NULL)
  {
    fwrite(encoded_data, sizeof(char), length, fp);
  }
}

int main()
{

  printf("Main: ---- start ----\n");

  fp = fopen("video.h264", "wb");

  scanf(mWidth, mHeight, initKbps, initFps, callback);

  sleep(2);

  st();

  fclose(fp);

  printf("Main: ---- finish ----\n");

  return 0;
}
