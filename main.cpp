#include <opencv2/viz/vizcore.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <termios.h>

using namespace cv;
using namespace std;
using namespace viz;

unsigned char start[11] = {0x01, 0x10, 0x00, 0x28, 0x00, 0x01, 0x02, 0x00, 0x04, 0xA1, 0xBB};
unsigned char stop[8] = {0x01, 0x06, 0x00, 0x28, 0x00, 0x00, 0x09, 0xC2};

Viz3d Window("Beacon");

volatile int gfd = 0;
unsigned int key = 0;

volatile int gv1 = 0;
volatile int gv2 = 0;

volatile int gv4 = 0;
volatile int gv5 = 0;
volatile int gv6 = 0;

void SerialSend(int fd, unsigned char *buf, int len)
{
    write(fd, buf, len);
}

void SerialRead(int fd, unsigned char *buf, int len, int *res_len)
{
    *res_len = read(fd, buf, len);
}

void inintBackground()
{
    Window.setBackgroundColor(Color::black());

    WLine xaxis(Point3f(0, 0, 0), Point3f(1, 0, 0), Color::red());
    xaxis.setRenderingProperty(LINE_WIDTH, 5);
    Window.showWidget("xaxis", xaxis);

    WLine yaxis(Point3f(0, 0, 0), Point3f(0, 1, 0), Color::green());
    yaxis.setRenderingProperty(LINE_WIDTH, 5);
    Window.showWidget("yaxis", yaxis);

    WLine zaxis(Point3f(0, 0, 0), Point3f(0, 0, 1), Color::blue());
    zaxis.setRenderingProperty(LINE_WIDTH, 5);
    Window.showWidget("zaxis", zaxis);

    WGrid grid(Vec2i::all(20), Vec2d::all(10), Color::white());
    Window.showWidget("grid", grid);
}

void KeyboardViz3d(const viz::KeyboardEvent &w, void *t)
{
    viz::Viz3d *fen = (viz::Viz3d *)t;
    if (w.action)
    {
        //       cout << "you pressed "<< w.code<<" = "<<w.symbol<< " in viz window "<<fen->getWindowName()<<"\n";
        switch (w.code)
        {
        case ' ':
            if (key++ % 2 == 0)
            {
                tcflush(gfd, TCOFLUSH);
                SerialSend(gfd, start, 11);
                WText text("Started", Point(0, 0), 20, Color::white());
                Window.showWidget("text", text);
            }
            else
            {
                tcflush(gfd, TCOFLUSH);
                SerialSend(gfd, stop, 8);
                WText text("Stopped", Point(0, 0), 20, Color::white());
                Window.showWidget("text", text);
            }
            break;
        }
    }
}

void updataRange(String name, double value)
{
    if ("AC" == name)
    {
        WCircle AC(value, 0.5, Color::red());
        Window.showWidget("AC", AC);
    }
    else if ("BC" == name)
    {
        WCircle BC(value, 0.5, Color::green());
        Window.showWidget("BC", BC);
        Mat VecB = Mat::zeros(1, 3, CV_32F);
        Mat MatB;
        Rodrigues(VecB, MatB);
        Affine3f poseB(MatB, Vec3f(100, 0, 0));
        Window.setWidgetPose("BC", poseB);
    }
    else if ("CC" == name)
    {
        WCircle CC(value, 0.5, Color::blue());
        Window.showWidget("CC", CC);
        Mat VecC = Mat::zeros(1, 3, CV_32F);
        Mat MatC;
        Rodrigues(VecC, MatC);
        Affine3f poseC(MatC, Vec3f(0, 100, 0));
        Window.setWidgetPose("CC", poseC);
    }
}

void inint()
{
    inintBackground();

    Window.registerKeyboardCallback(KeyboardViz3d, &Window);

    WCylinder A(Point3d(0, 0, 0), Point3d(0, 0, 20), 1, 10, Color::red());
    Window.showWidget("A", A);

    WCylinder B(Point3d(100, 0, 0), Point3d(100, 0, 20), 1, 10, Color::green());
    Window.showWidget("B", B);

    WCylinder C(Point3d(0, 100, 0), Point3d(0, 100, 20), 1, 10, Color::blue());
    Window.showWidget("C", C);

    updataRange("AC", 0);
    updataRange("BC", 0);
    updataRange("CC", 0);
}

void processData(unsigned char *buf, int len)
{
    if (len != 31)
    {
        return;
    }
    int v1 = (int)(((buf[7] << 8) & 0xFF00) | buf[8]);
    int v2 = (int)(((buf[9] << 8) & 0xFF00) | buf[10]);
    int v3 = (int)((((buf[11] << 8) & 0xFF00) | buf[12]));
    int v4 = (int)(((buf[13] << 8) & 0xFF00) | buf[14]);
    int v5 = (int)((((buf[15] << 8) & 0xFF00) | buf[16]));
    int v6 = (int)(((buf[17] << 8) & 0xFF00) | buf[18]);
    int v7 = (int)(((buf[19] << 8) & 0xFF00) | buf[20]);
    int v8 = (int)(((buf[21] << 8) & 0xFF00) | buf[22]);
    int v9 = (int)(((buf[23] << 8) & 0xFF00) | buf[24]);
    int v10 = (int)(((buf[25] << 8) & 0xFF00) | buf[26]);
    int v11 = (int)(((buf[27] << 8) & 0xFF00) | buf[28]);

    printf("%d ", v1);
    printf("%d ", v2);
    printf("%d ", v3);
    printf("%d ", v4);
    printf("%d ", v5);
    printf("%d ", v6);
    printf("%d ", v7);
    printf("%d ", v8);
    printf("%d ", v9);
    printf("%d ", v10);
    printf("%d ", v11);

    gv1 = v1;
    gv2 = v2;

    gv4 = v4;
    gv5 = v5;
    gv6 = v6;

    printf("\n");
    fflush(stdout);
}

void *thread(void *args)
{
    int fd;
    gfd = fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);

    printf("fd=%d\n", fd);

    if (fd == -1)
    {
        printf("open port fail\n");
        return 0;
    }
    else
    {
        printf("open port success\n");
    }

    struct termios SerialPortSettings;
    tcgetattr(fd, &SerialPortSettings);
    cfsetispeed(&SerialPortSettings, B115200);
    cfsetospeed(&SerialPortSettings, B115200);

    /* 8N1 Mode */
    SerialPortSettings.c_cflag &= ~HUPCL;
    SerialPortSettings.c_iflag &= ~INPCK;
    SerialPortSettings.c_iflag |= IGNBRK;
    SerialPortSettings.c_iflag &= ~ICRNL;
    SerialPortSettings.c_iflag &= ~IXON;
    SerialPortSettings.c_lflag &= ~IEXTEN;
    SerialPortSettings.c_lflag &= ~ECHOK;
    SerialPortSettings.c_lflag &= ~ECHOCTL;
    SerialPortSettings.c_lflag &= ~ECHOKE;
    SerialPortSettings.c_oflag &= ~ONLCR;
    SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    SerialPortSettings.c_oflag &= ~OPOST;

    /* Setting Time outs */
    SerialPortSettings.c_cc[VMIN] = 1;
    SerialPortSettings.c_cc[VTIME] = 31;

    if ((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0)
    {
        printf("Setting fail\n");
        return 0;
    }
    else
    {
        printf("Setting success\n");
    }

    //    tcflush(fd, TCIFLUSH);

    while (1)
    {
        unsigned char read_buffer[32];
        int len = 0;
        SerialRead(fd, read_buffer, 32, &len);
        cout << "SerialRead" << endl;
        processData(read_buffer, len);
        cout << "processData" << endl;
    }

    close(fd);
    cout << "thread end" << endl;
}

int main()
{
    inint();

    updataRange("AC", 1);
    updataRange("BC", 2);
    updataRange("CC", 3);

    WSphere tar(Point3d(1, 1, 0), 0.3, 10, Color::yellow());
    Window.showWidget("tar", tar);

    pthread_t tids;

    int ret = pthread_create(&tids, NULL, thread, NULL);
    if (ret != 0)
    {
        cout << "pthread_create error: error_code=" << ret << endl;
    }

    while (1)
    {
        updataRange("AC", gv4);
        updataRange("BC", gv5);
        updataRange("CC", gv6);

        WSphere tar(Point3d(gv1, gv2, 0), 5, 10, Color::yellow());
        Window.showWidget("tar", tar);

        Window.spinOnce();
    }

    pthread_exit(NULL);
}
