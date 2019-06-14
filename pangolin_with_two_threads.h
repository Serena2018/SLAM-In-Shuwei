//
// Created by tp on 6/14/19.
//
#include <iostream>
#include <zconf.h>
#include <pthread.h>
#include <cv.h>

#include <sophus/se3.h>
#include <pangolin/pangolin.h>

#ifndef TRAJECTORY_COMPARE_PANGOLIN_WITH_TWO_THREADS_H
#define TRAJECTORY_COMPARE_PANGOLIN_WITH_TWO_THREADS_H

using namespace std;
using namespace cv;

//Define a global variable as the share data container
vector<Eigen::Vector3f> wifilocations;
bool flag_end(false);

//Function for drawing camera trajectory, sparse map points and wifi points
void Trajectory_PointCloud_Wifi_Map(vector<Eigen::Vector3f> trajectory,vector<Eigen::Vector3f> pointcloud, vector<Eigen::Vector3f> &wifilocations)
{

    if (trajectory.empty() || pointcloud.empty() || wifilocations.empty())
    {
        cerr << "Trajectory or pointcloud is empty, please check your file!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory and pointcloud
    pangolin::CreateWindowAndBind("SeeMap", 1024, 768);

    glEnable(GL_DEPTH_TEST);

    glEnable(GL_BLEND);

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            //对应的是gluLookAt,摄像机位置,参考点位置,up vector(上向量)
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        pangolin::glDrawAxis(3);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        //Draw camera trajectory
        glLineWidth(8);
        for (size_t i = 0; i < trajectory.size() - 1-1; i++)
        {

            glColor3f(0.0,1.0,0.0);
            glBegin(GL_LINES);
            auto p1 = trajectory[i], p2 = trajectory[i + 1];
            glVertex3d(p1[0], p1[1], p1[2]);
            glVertex3d(p2[0], p2[1], p2[2]);

            glEnd();
        }

        //Draw sparse map points
        glPointSize(2);
        for (size_t j = 0; j < pointcloud.size() - 1; j++)
        {
            glColor3f(1.0,0.0,0.0);

            glBegin(GL_POINTS);
            Eigen::Vector3f position = pointcloud[j];
            glVertex3d(position[0], position[1], position[2]);

            glEnd();
        }
        //Draw wifi points
        Eigen::Vector3f wifilocation;

        for (int i=0; i < wifilocations.size()-1; ++i)
        {
            wifilocation = wifilocations[i];
            glPointSize(5);
            glColor3f(0.0,0.0,0.0);
            glBegin(GL_POINTS);
            glVertex3d(wifilocation[0]+5, wifilocation[1]+5, wifilocation[2]+5);
            glEnd();
        }
        //Draw the newest point in different color
        wifilocation = wifilocations[wifilocations.size()-1];
        glPointSize(5);
        glColor3f(0.0,1.0,1.0);
        glBegin(GL_POINTS);
        glVertex3d(wifilocation[0]+5, wifilocation[1]+5, wifilocation[2]+5);
        glEnd();
        pangolin::FinishFrame();
    }
}
int split(std::string str, std::string pattern, std::vector<std::string> &words)
{
    std::string::size_type pos;
    std::string word;
    int num = 0;
    str += pattern;
    std::string::size_type size = str.size();
    for (auto i = 0; i < size; i++) {
        pos = str.find(pattern, i);
        if (pos == i) {
            continue;//if first string is pattern
        }
        if (pos < size) {
            word = str.substr(i, pos - i);
            words.push_back(word);
            i = pos + pattern.size() - 1;
            num++;
        }
    }
    return num;
}

//Load .obj file
void loadObjFile(std::string filename, std::vector<Eigen::Vector3f> &v_array)
{
    std::ifstream fin(filename);
    std::string str;
    std::vector<std::string> words;
    Eigen::Vector3f v;
    int num = 0;
    while (!fin.eof())
    {
        std::getline(fin, str);
        words.clear();
        num = split(str, " ", words);
        //std::cout << "num = " << num << std::endl;
        //getchar();
        if (num == 4 && words[0] == "v")
        {
            v[0] = atof(words[1].c_str());
            v[1] = atof(words[2].c_str());
            v[2] = atof(words[3].c_str());
            v_array.push_back(v);
        }
    }
}


#endif //TRAJECTORY_COMPARE_PANGOLIN_WITH_TWO_THREADS_H
