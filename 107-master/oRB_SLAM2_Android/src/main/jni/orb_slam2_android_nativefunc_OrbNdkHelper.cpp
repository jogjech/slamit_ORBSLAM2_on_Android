#include "orb_slam2_android_nativefunc_OrbNdkHelper.h"
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <errno.h>
#include <GLES/gl.h>
#include <android/asset_manager_jni.h>
#include<opencv2/core/core.hpp>
#include<System.h>
#include<math.h>
#include<Converter.h>
using namespace cv;
#include <android/log.h>
#define LOG_TAG "ORB_SLAM_SYSTEM"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define LOG(...) __android_log_print(ANDROID_LOG_ERROR,LOG_TAG, __VA_ARGS__)
static ORB_SLAM2::System *s;
bool init_end = false;

/*
 * Class:     orb_slam2_android_nativefunc_OrbNdkHelper
 * Method:    initSystemWithParameters
 * Signature: (Ljava/lang/String;Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_orb_slam2_android_nativefunc_OrbNdkHelper_initSystemWithParameters
(JNIEnv * env, jclass cls, jstring VOCPath, jstring calibrationPath) {
	const char *calChar = env->GetStringUTFChars(calibrationPath, JNI_FALSE);
	const char *vocChar = env->GetStringUTFChars(VOCPath, JNI_FALSE);
	// use your string
	std::string voc_string(vocChar);
	std::string cal_string(calChar);
	env->GetJavaVM(&jvm);
	jvm->AttachCurrentThread(&env, NULL);
	s=new ORB_SLAM2::System(voc_string,cal_string,ORB_SLAM2::System::MONOCULAR,true);
	env->ReleaseStringUTFChars(calibrationPath, calChar);
	env->ReleaseStringUTFChars(VOCPath, vocChar);
	init_end=true;
}

/*
 * Class:     orb_slam2_android_nativefunc_OrbNdkHelper
 * Method:    startCurrentORB
 * Signature: (DDD[I)[I
 */
JNIEXPORT jintArray JNICALL Java_orb_slam2_android_nativefunc_OrbNdkHelper_startCurrentORB(
		JNIEnv * env, jclass cls, jdouble curTimeStamp, jintArray buf, jint w,
		jint h) {
	jint *cbuf;
	cbuf = env->GetIntArrayElements(buf, false);
	if (cbuf == NULL) {
		return 0;
	}
	int size = w * h;
	cv::Mat myimg(h, w, CV_8UC4, (unsigned char*) cbuf);
	cv::Mat ima = s->TrackMonocular(myimg, curTimeStamp);
	jintArray resultArray = env->NewIntArray(ima.rows * ima.cols);
	jint *resultPtr;
	resultPtr = env->GetIntArrayElements(resultArray, false);
	for (int i = 0; i < ima.rows; i++)
		for (int j = 0; j < ima.cols; j++) {
			int R = ima.at < Vec3b > (i, j)[0];
			int G = ima.at < Vec3b > (i, j)[1];
			int B = ima.at < Vec3b > (i, j)[2];
			resultPtr[i * ima.cols + j] = 0xff000000 + (R << 16) + (G << 8) + B;
		}
	env->ReleaseIntArrayElements(resultArray, resultPtr, 0);
	env->ReleaseIntArrayElements(buf, cbuf, 0);
	return resultArray;
}
/*
 * Class:     orb_slam2_android_nativefunc_OrbNdkHelper
 * Method:    glesInit
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_orb_slam2_android_nativefunc_OrbNdkHelper_glesInit
(JNIEnv *env, jclass cls) {
	// 启用阴影平滑
	glShadeModel(GL_SMOOTH);
	// 黑色背景
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	// 设置深度缓存
	glClearDepthf(1.0f);
	// 启用深度测试
	glEnable(GL_DEPTH_TEST);
	// 所作深度测试的类型
	glDepthFunc(GL_LEQUAL);
	// 告诉系统对透视进行修正
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}

/*
 * Class:     orb_slam2_android_nativefunc_OrbNdkHelper
 * Method:    glesRender
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_orb_slam2_android_nativefunc_OrbNdkHelper_glesRender
(JNIEnv * env, jclass cls) {
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();
	//glScalef (1.0, -1.0, 1.0);
	if(init_end)
	s->drawGL();
}

/*
 * Class:     orb_slam2_android_nativefunc_OrbNdkHelper
 * Method:    glesResize
 * Signature: (II)V
 */
JNIEXPORT void JNICALL Java_orb_slam2_android_nativefunc_OrbNdkHelper_glesResize
(JNIEnv *env, jclass cls, jint width, jint height) {
	//图形最终显示到屏幕的区域的位置、长和宽
	glViewport (0,0,width,height);
	//指定矩阵
	glMatrixMode (GL_PROJECTION);
	//将当前的矩阵设置为glMatrixMode指定的矩阵
	glLoadIdentity ();
	glOrthof(-2, 2, 2, -2, -2, 2);
}

/*
 * Class:     orb_slam2_android_nativefunc_OrbNdkHelper
 * Method:    readShaderFile
 * Signature: (Landroid/content/res/AssetManager;)V
 */
JNIEXPORT jfloatArray JNICALL Java_orb_slam2_android_nativefunc_OrbNdkHelper_startCurrentORBForCamera
(JNIEnv *env, jclass cls,jdouble timestamp, jlong addr,jint w,jint h) {
/*	const cv::Mat *im = (cv::Mat *) addr;
	cv::Mat ima = s->TrackMonocular(*im, timestamp);
	jintArray resultArray = env->NewIntArray(ima.rows * ima.cols);
	jint *resultPtr;
	resultPtr = env->GetIntArrayElements(resultArray, false);
	for (int i = 0; i < ima.rows; i++)
	for (int j = 0; j < ima.cols; j++) {
		int R = ima.at < Vec3b > (i, j)[0];
		int G = ima.at < Vec3b > (i, j)[1];
		int B = ima.at < Vec3b > (i, j)[2];
		resultPtr[i * ima.cols + j] = 0xff000000 + (R << 16) + (G << 8) + B;
	}
	env->ReleaseIntArrayElements(resultArray, resultPtr, 0);
	return resultArray;*/
	const cv::Mat *im = (cv::Mat *) addr;
    	cv::Mat ima = s->TrackMonocular(*im, timestamp);
    	//maxiaoba
   	            /*cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
                cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); //origin to camera in world frame
                vector<float> q = Converter::toQuaternion(Rwc);
                vector<float> rpy(3);
                rpy[0] = atan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2]));
                rpy[1] = asin(2*(q[0]*q[2]-q[3]*q[1]));
                rpy[2] = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
                vector<float> p(3);
                p[0] = twc.at <float> (0);
                p[1] = twc.at <float> (1);
                p[2] = twc.at <float> (2);*/
    	//maxiaoba
    	jfloatArray resultArray = env->NewFloatArray(ima.rows * ima.cols);
                 	jfloat *resultPtr;
                     resultPtr = env->GetFloatArrayElements(resultArray, false);
                     	for (int i = 0; i < ima.rows; i++)
                     	for (int j = 0; j < ima.cols; j++) {

                     		resultPtr[i * ima.cols + j] = ima.at <float> (i,j);
                     	}
                     	/*jfloatArray resultArray = env->NewFloatArray(6);
                     	jfloat *resultPtr;
                        resultPtr = env->GetFloatArrayElements(resultArray, false);
                        resultPtr[0] = p[0];
                        resultPtr[1] = p[1];
                        resultPtr[2] = p[2];
                        resultPtr[3] = rpy[0];
                        resultPtr[4] = rpy[1];
                        resultPtr[5] = rpy[2];*/

        env->ReleaseFloatArrayElements(resultArray, resultPtr, 0);
    	return resultArray;
}

//maxiaoba
JNIEXPORT void JNICALL Java_orb_slam2_android_nativefunc_OrbNdkHelper_trackOnly
  (JNIEnv *env, jclass cls) {
        s->ActivateLocalizationMode();
  }
JNIEXPORT jfloatArray JNICALL Java_orb_slam2_android_nativefunc_OrbNdkHelper_startCurrentORBForCamera2
      (JNIEnv *env, jclass cls,jdouble timestamp, jlong addr,jint w,jint h,jfloatArray R){
        const cv::Mat *im = (cv::Mat *) addr;
        jfloat* Rdummy = env->GetFloatArrayElements(R, false);
    	cv::Mat Reb = cv::Mat::eye(4,4,CV_32F);
        for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            Reb.at <float> (i,j) = Rdummy[i * 3 + j];
        }
        cv::Mat ima = s->TrackMonocular(*im, timestamp,s->getmVelocity(Reb));
    	//maxiaoba
   	            /*cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
                cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); //origin to camera in world frame
                vector<float> q = Converter::toQuaternion(Rwc);
                vector<float> rpy(3);
                rpy[0] = atan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2]));
                rpy[1] = asin(2*(q[0]*q[2]-q[3]*q[1]));
                rpy[2] = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
                vector<float> p(3);
                p[0] = twc.at <float> (0);
                p[1] = twc.at <float> (1);
                p[2] = twc.at <float> (2);*/
    	//maxiaoba
    	jfloatArray resultArray = env->NewFloatArray(ima.rows * ima.cols);
                 	jfloat *resultPtr;
                     resultPtr = env->GetFloatArrayElements(resultArray, false);
                     	for (int i = 0; i < ima.rows; i++)
                     	for (int j = 0; j < ima.cols; j++) {

                     		resultPtr[i * ima.cols + j] = ima.at <float> (i,j);
                     	}
                     	/*jfloatArray resultArray = env->NewFloatArray(6);
                     	jfloat *resultPtr;
                        resultPtr = env->GetFloatArrayElements(resultArray, false);
                        resultPtr[0] = p[0];
                        resultPtr[1] = p[1];
                        resultPtr[2] = p[2];
                        resultPtr[3] = rpy[0];
                        resultPtr[4] = rpy[1];
                        resultPtr[5] = rpy[2];*/

        env->ReleaseFloatArrayElements(resultArray, resultPtr, 0);
    	return resultArray;
      }
//maxiaoba
JNIEXPORT void JNICALL Java_orb_slam2_android_nativefunc_OrbNdkHelper_dataFusion
  (JNIEnv *, jclass,jdouble curTimeStamp, jdouble lat ,jdouble lng, jdouble accex, jdouble accey,jdouble accez,jdouble gyrox, jdouble gyroy,jdouble gyroz){

  }