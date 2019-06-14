//
// Created by tp on 6/13/19.
//
////
//// Created by tp on 6/13/19.
////


#include <pangolin_with_two_threads.h>

//The first thread for load files of camera trajectory and sparse map points, moreover exacute the Draw function for drawing all information
void *Map_Drawer(void *arg)
{
    std::cout<<"The first thread"<<std::endl;

    vector<Eigen::Vector3f> trajectory;

    ifstream infile;

    infile.open("../CameraTrajectory_KITTI.txt");

    if(!infile)
    {
        cout<<"error data"<<endl;
        return 0;
    }

    int lineCnt=0;
    string str;
    while (!infile.eof())
    {
        std::getline(infile, str);
        if (str != "")
        {
            lineCnt++;
        }
    }
    lineCnt = lineCnt+1;

    cout<<"存入数组"<<endl;   //先将文件中的数据存入到一个二维数组中
    double data;
    double a[lineCnt][12];
    double *p=&a[0][0];
    infile.clear();
    infile.seekg(ios::beg);
    while(infile>>data)             //遇到空白符结束
    {
        *p=data;
        p++;
    }
    infile.close();

    for(int i=0;i<lineCnt;i++)   //分别对每一行数据生成一个变换矩阵，然后存入动态数组poses中
    {
        Eigen::Vector3f t1;
        t1<<a[i][3],a[i][7],a[i][11];
        trajectory.push_back(t1);
        std::cout<<"saved location"<<":"<<t1[0]<<" "<<t1[1]<<" "<<t1[2]<<endl;
    }

    std::vector<Eigen::Vector3f> pointcloud;

    loadObjFile("../pointcloud.obj", pointcloud);

    // draw trajectory in pangolin
    cout << "wifilocations size---- = " << wifilocations.size() <<endl;
    Trajectory_PointCloud_Wifi_Map(trajectory, pointcloud, wifilocations);
    return 0;
}
//The second thread is for loading wifi points into the share data container.
void *WiFi_Drawer(void *arg)
{
    std::cout<<"The second thread"<<std::endl;

    //vector<Eigen::Vector3f> WIFI;

    ifstream infile;

    infile.open("../CameraTrajectory_KITTI.txt");

    if(!infile)
    {
        cout<<"error data"<<endl;
        return 0;
    }

    int lineCnt=0;
    string str;
    while (!infile.eof())
    {
        std::getline(infile, str);
        if (str != "")
        {
            lineCnt++;
        }
    }
    lineCnt = lineCnt+1;

    cout<<"存入数组"<<endl;   //先将文件中的数据存入到一个二维数组中
    double data;
    double a[lineCnt][12];
    double *p=&a[0][0];
    ///在上面通过读取文件的行数来统计文件中共有多少行的时候，文件指针已经到达了文件末尾，
    ///所以如果要继续读取文件内容，则需要将文件指针设定指在文件的开始。
    infile.clear();
    infile.seekg(ios::beg);

    while(infile>>data)             //遇到空白符结束
    {
        *p=data;
        p++;
    }
    infile.close();

    for(int i=0;i<lineCnt;i++)   //分别对每一行数据生成一个变换矩阵，然后存入动态数组poses中
    {
        Eigen::Vector3f t1;
        t1<<a[i][3],a[i][7],a[i][11];
        wifilocations.push_back(t1);
        sleep(1);
    }
}

int main(int argc, char** argv)
{
    std::cout << "hello" << std::endl;

    int ret = 0;

    pthread_t tid0;
    ret = pthread_create(&tid0,NULL,Map_Drawer, NULL); //attr == NULL表示默认属性(可结合属性)
    printf("thread 0 create succeed! \n");

    if(ret!=0)
    {
        perror("pthread_create 0 fail");
        exit(EXIT_SUCCESS);
    }
    pthread_t tid1;
    ret = pthread_create(&tid1,NULL,WiFi_Drawer,NULL); //attr == NULL表示默认属性(可结合属性)
    printf("thread 1 create succeed! \n");

    if(ret!=0)
    {
        perror("pthread_create 1 fail");
        exit(EXIT_SUCCESS);
    }
    int timecount=0;

    while(!flag_end)
    {

        sleep(5);
        timecount+=1;
        std::cout<<"the process going-------------- " << timecount <<std::endl;
    }
    return 0;
}


