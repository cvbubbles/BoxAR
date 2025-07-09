#pragma once
#pragma once
#ifndef MGLRENDER_H
#define MGLRENDER_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <opencv2/opencv.hpp>

#include <glrender/shader_m.h>
#include <glrender/model.h>
#include <glrender/camera.h>

#include <iostream>

void framebuffer_size_callback(GLFWwindow* window, int width, int height);

cv::Mat saveImageUsingCV(cv::Mat& bgImg, int w, int h, cv::Mat1f& bgDepth)
{
    GLubyte* pPixelData;
    pPixelData = (GLubyte*)malloc(w * h * 4);
    if (pPixelData == 0)
    {
        std::cout << "No render result can be saved" << std::endl;
        exit(-1);
    }
    glReadBuffer(GL_FRONT);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, pPixelData);

    cv::Mat img;
    std::vector<cv::Mat> imgPlanes;
    img.create(h, w, CV_8UC3);
    cv::split(img, imgPlanes);

    for (int i = 0; i < h; i++) {
        unsigned char* plane0Ptr = imgPlanes[0].ptr<unsigned char>(i);//B
        unsigned char* plane1Ptr = imgPlanes[1].ptr<unsigned char>(i);//G
        unsigned char* plane2Ptr = imgPlanes[2].ptr<unsigned char>(i);//R

        for (int j = 0; j < w; j++) {
            int k = 4 * (i * w + j);//RGBA and skip A
            plane2Ptr[j] = pPixelData[k];//R
            plane1Ptr[j] = pPixelData[k + 1];//G
            plane0Ptr[j] = pPixelData[k + 2];//B
        }
    }
    cv::merge(imgPlanes, img);
    cv::flip(img, img, 0);

    float* pDepthData;
    pDepthData = new float[w * h];
    glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, pDepthData);
    cv::Mat depthImg;
    depthImg.create(h, w, CV_32FC1);
    for (int i = 0; i < h; i++) {
        float* depthPtr = depthImg.ptr<float>(i);
        for (int j = 0; j < w; j++) {
            int k = i * w + j;
            depthPtr[j] = pDepthData[k];
        }
    }
    cv::flip(depthImg, depthImg, 0);
    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            if (depthImg.at<float>(i, j) == 1.0f || depthImg.at<float>(i, j) > bgDepth.at<float>(i, j))
            {
                img.at<cv::Vec3b>(i, j) = bgImg.at<cv::Vec3b>(i, j);
            }
        }
    }
    // cv::imshow("img", img);
     //cv::waitKey(0);
    return img;
}

void glRender(const std::string& obj_path,
    const cv::Matx44f& cv_view, const cv::Matx44f& cv_projection,
    int width, int height)
{

    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(width, height, "", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        exit(-1);
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {

        std::cout << "Failed to initialize GLAD" << std::endl;
        exit(-1);
    }

    // tell stb_image.h to flip loaded texture's on the y-axis (before loading model).

    stbi_set_flip_vertically_on_load(true);

    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);// add phong
    // build and compile shaders
    // -------------------------
    Shader testShader("vertex.glsl", "frag.glsl");

    MModel mModel(obj_path);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    // render loop
    // -----------
   // while (!glfwWindowShouldClose(window))
    cv::Mat res;
    {
        // render
        // ------
        glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // don't forget to enable shader before setting uniforms
        testShader.use();

        glm::mat4 projection = glm::mat4(1.0f);
        glm::mat4 view = glm::mat4(1.0f);

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                view[i][j] = cv_view(i, j);
                projection[i][j] = cv_projection(i, j);
            }
        }
        testShader.setMat4("projection", projection);
        testShader.setMat4("view", view);


        // render the loaded model
        glm::mat4 model = glm::mat4(1.0f);

        testShader.setMat4("model", model);
        mModel.Draw(testShader);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    //glfwTerminate();
}
// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

#endif