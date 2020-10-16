#ifdef OPENCV

#include "stdio.h"
#include "stdlib.h"
#include "opencv2/opencv.hpp"
#include "image.h"

using namespace cv;

extern "C" {

IplImage *image_to_ipl(image im)
{
    int x,y,c;
    IplImage *disp = cvCreateImage(cvSize(im.w,im.h), IPL_DEPTH_8U, im.c);
    int step = disp->widthStep;
    for(y = 0; y < im.h; ++y){
        for(x = 0; x < im.w; ++x){
            for(c= 0; c < im.c; ++c){
                float val = im.data[c*im.h*im.w + y*im.w + x];
                disp->imageData[y*step + x*im.c + c] = (unsigned char)(val*255);
            }
        }
    }
    return disp;
}

image ipl_to_image(IplImage* src)
{
    int h = src->height;
    int w = src->width;
    int c = src->nChannels;
    image im = make_image(w, h, c);
    unsigned char *data = (unsigned char *)src->imageData;
    int step = src->widthStep;
    int i, j, k;

    for(i = 0; i < h; ++i){
        for(k= 0; k < c; ++k){
            for(j = 0; j < w; ++j){
                im.data[k*w*h + i*w + j] = data[i*step + j*c + k]/255.;
            }
        }
    }
    return im;
}

Mat image_to_mat(image im)
{
    image copy = copy_image(im);
    constrain_image(copy);
    if(im.c == 3) rgbgr_image(copy);

    IplImage *ipl = image_to_ipl(copy);
    Mat m = cvarrToMat(ipl, true);
    cvReleaseImage(&ipl);
    free_image(copy);
    return m;
}

image mat_to_image(Mat m)
{
    IplImage ipl = m;
    image im = ipl_to_image(&ipl);
    rgbgr_image(im);
    return im;
}

void *open_video_stream(const char *f, int c, int w, int h, int fps)
{
    VideoCapture *cap;
    if(f) cap = new VideoCapture(f);
    else cap = new VideoCapture(c);
    if(!cap->isOpened()) return 0;
    if(w) cap->set(CV_CAP_PROP_FRAME_WIDTH, w);
    if(h) cap->set(CV_CAP_PROP_FRAME_HEIGHT, w);
    if(fps) cap->set(CV_CAP_PROP_FPS, w);
    return (void *) cap;
}

image get_image_from_stream(void *p)
{
    VideoCapture * cap = (VideoCapture *)p;
    Mat m;

    *cap >> m;
    
    if(m.empty()) return make_empty_image(0,0,0);
    return mat_to_image(m);
}

#ifdef _JB_EDIT_
image get_image_form_slam_shm(void)
{
    Mat img(480, 640, CV_8UC3);
    int shmid = 0;
    RGB_DATA *shm_info= NULL;
    void *shared_memory = (void *)0;

    shmid = shmget((key_t)4003, sizeof(RGB_DATA), 0666|IPC_CREAT);

    if (shmid == -1)
    {
        perror("shmget failed : ");
        exit(0);
    }

    shared_memory = shmat(shmid, (void *)0, 0666|IPC_CREAT);

    if (shared_memory == (void *)-1)
    {
        perror("shmat attach is failed : ");
        exit(0);
    }

    shm_info = (RGB_DATA*)shared_memory;

    for (int y = 0; y < ROW_SIZE; y++) {

      	// y번째 row에 대한 주소를 포인터에 저장한 후
	uchar* pointer_input = img.ptr<uchar>(y);

	for (int x = 0; x < COL_SIZE; x++) {
	    // row 포인터로부터 (x * 3 )번째 떨어져 있는 픽셀을 가져옵니다.
	    //0, 1, 2 순서대로 blue, green, red 채널값을 가져올 수있는 이유는 하나의 픽셀이 메모리상에 b g r 순서대로 저장되기 때문입니다. 
	    pointer_input[x * 3 + 0] = shm_info->b[y][x];
	    pointer_input[x * 3 + 1] = shm_info->g[y][x];
	    pointer_input[x * 3 + 2] = shm_info->r[y][x];
	}
    }

    if(img.empty()) return make_empty_image(0,0,0);
    return mat_to_image(img);
}
#endif

image load_image_cv(char *filename, int channels)
{
    int flag = -1;
    if (channels == 0) flag = -1;
    else if (channels == 1) flag = 0;
    else if (channels == 3) flag = 1;
    else {
        fprintf(stderr, "OpenCV can't force load with %d channels\n", channels);
    }
    Mat m;
    m = imread(filename, flag);
    if(!m.data){
        fprintf(stderr, "Cannot load image \"%s\"\n", filename);
        char buff[256];
        sprintf(buff, "echo %s >> bad.list", filename);
        system(buff);
        return make_image(10,10,3);
        //exit(0);
    }
    image im = mat_to_image(m);
    return im;
}

int show_image_cv(image im, const char* name, int ms)
{
    Mat m = image_to_mat(im);
    imshow(name, m);
    int c = waitKey(ms);
    if (c != -1) c = c%256;
    return c;
}

void make_window(char *name, int w, int h, int fullscreen)
{
    namedWindow(name, WINDOW_NORMAL); 
    if (fullscreen) {
        setWindowProperty(name, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    } else {
        resizeWindow(name, w, h);
        if(strcmp(name, "Demo") == 0) moveWindow(name, 0, 0);
    }
}

}

#endif
