//=============================================================================
// Copyright (c) 2001-2021 FLIR Systems, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of FLIR
// Integrated Imaging Solutions, Inc. ("Confidential Information"). You
// shall not disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with FLIR Integrated Imaging Solutions, Inc. (FLIR).
//
// FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================

/**
 *  @example Acquisition.cpp
 *
 *  @brief Acquisition.cpp shows how to acquire images. It relies on
 *  information provided in the Enumeration example. Also, check out the
 *  ExceptionHandling and NodeMapInfo examples if you haven't already.
 *  ExceptionHandling shows the handling of standard and Spinnaker exceptions
 *  while NodeMapInfo explores retrieving information from various node types.
 *
 *  This example touches on the preparation and cleanup of a camera just before
 *  and just after the acquisition of images. Image retrieval and conversion,
 *  grabbing image data, and saving images are all covered as well.
 *
 *  Once comfortable with Acquisition, we suggest checking out
 *  AcquisitionMultipleCamera, NodeMapCallback, or SaveToAvi.
 *  AcquisitionMultipleCamera demonstrates simultaneously acquiring images from
 *  a number of cameras, NodeMapCallback serves as a good introduction to
 *  programming with callbacks and events, and SaveToAvi exhibits video creation.
 */
#include <fstream>
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sys/time.h>
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

#ifdef _DEBUG
// Disables heartbeat on GEV cameras so debugging does not incur timeout errors
int DisableHeartbeat(INodeMap& nodeMap, INodeMap& nodeMapTLDevice)
{
    cout << "Checking device type to see if we need to disable the camera's heartbeat..." << endl << endl;

    //
    // Write to boolean node controlling the camera's heartbeat
    //
    // *** NOTES ***
    // This applies only to GEV cameras and only applies when in DEBUG mode.
    // GEV cameras have a heartbeat built in, but when debugging applications the
    // camera may time out due to its heartbeat. Disabling the heartbeat prevents
    // this timeout from occurring, enabling us to continue with any necessary debugging.
    // This procedure does not affect other types of cameras and will prematurely exit
    // if it determines the device in question is not a GEV camera.
    //
    // *** LATER ***
    // Since we only disable the heartbeat on GEV cameras during debug mode, it is better
    // to power cycle the camera after debugging. A power cycle will reset the camera
    // to its default settings.
    //
    CEnumerationPtr ptrDeviceType = nodeMapTLDevice.GetNode("DeviceType");
    if (!IsAvailable(ptrDeviceType) || !IsReadable(ptrDeviceType))
    {
        cout << "Error with reading the device's type. Aborting..." << endl << endl;
        return -1;
    }
    else
    {
        if (ptrDeviceType->GetIntValue() == DeviceType_GigEVision)
        {
            cout << "Working with a GigE camera. Attempting to disable heartbeat before continuing..." << endl << endl;
            CBooleanPtr ptrDeviceHeartbeat = nodeMap.GetNode("GevGVCPHeartbeatDisable");
            if (!IsAvailable(ptrDeviceHeartbeat) || !IsWritable(ptrDeviceHeartbeat))
            {
                cout << "Unable to disable heartbeat on camera. Continuing with execution as this may be non-fatal..."
                     << endl
                     << endl;
            }
            else
            {
                ptrDeviceHeartbeat->SetValue(true);
                cout << "WARNING: Heartbeat on GigE camera disabled for the rest of Debug Mode." << endl;
                cout << "         Power cycle camera when done debugging to re-enable the heartbeat..." << endl << endl;
            }
        }
        else
        {
            cout << "Camera does not use GigE interface. Resuming normal execution..." << endl << endl;
        }
    }
    return 0;
}
#endif

unsigned long long msse = 0;


// This function configures a number of settings on the camera including offsets
// X and Y, width, height, and pixel format. These settings must be applied before
// BeginAcquisition() is called; otherwise, they will be read only. Also, it is
// important to note that settings are applied immediately. This means if you plan
// to reduce the width and move the x offset accordingly, you need to apply such
// changes in the appropriate order.
int ConfigureCustomImageSettings(Spinnaker::GenApi::INodeMap& nodeMap)
{
    
    
    int result = 0;

    std::cout << endl << endl << "*** CONFIGURING CUSTOM IMAGE SETTINGS ***" << endl << endl;

    try
    {
        //
        // Apply mono 8 pixel format
        //
        // *** NOTES ***
        // Enumeration nodes are slightly more complicated to set than other
        // nodes. This is because setting an enumeration node requires working
        // with two nodes instead of the usual one.
        //
        // As such, there are a number of steps to setting an enumeration node:
        // retrieve the enumeration node from the nodemap, retrieve the desired
        // entry node from the enumeration node, retrieve the integer value from
        // the entry node, and set the new value of the enumeration node with
        // the integer value from the entry node.
        //
        // Retrieve the enumeration node from the nodemap
        Spinnaker::GenApi::CEnumerationPtr ptrPixelFormat = nodeMap.GetNode("PixelFormat");
        if (Spinnaker::GenApi::IsAvailable(ptrPixelFormat) && Spinnaker::GenApi::IsWritable(ptrPixelFormat))
        {
            // Retrieve the desired entry node from the enumeration node
            Spinnaker::GenApi::CEnumEntryPtr ptrPixelFormatBayerRG8 = ptrPixelFormat->GetEntryByName("BayerRG8");
            if (Spinnaker::GenApi::IsAvailable(ptrPixelFormatBayerRG8) && Spinnaker::GenApi::IsReadable(ptrPixelFormatBayerRG8))
            {
                // Retrieve the integer value from the entry node
                int64_t pixelFormatBayerRG8 = ptrPixelFormatBayerRG8->GetValue();

                // Set integer as new value for enumeration node
                ptrPixelFormat->SetIntValue(pixelFormatBayerRG8);

                cout << "Pixel format set to " << ptrPixelFormat->GetCurrentEntry()->GetSymbolic() << "..." << endl;
            }
            else
            {
                cout << "Pixel format Bayer RG8 not available..." << endl;
            }
        }
        else
        {
            cout << "Pixel format not available..." << endl;
        }

        //
        // Apply minimum to offset X
        //
        // *** NOTES ***
        // Numeric nodes have both a minimum and maximum. A minimum is retrieved
        // with the method GetMin(). Sometimes it can be important to check
        // minimums to ensure that your desired value is within range.
        //
        Spinnaker::GenApi::CIntegerPtr ptrOffsetX = nodeMap.GetNode("OffsetX");
        if (IsAvailable(ptrOffsetX) && IsWritable(ptrOffsetX))
        {
            //ptrOffsetX->SetValue((ptrOffsetX->GetMin() + ptrOffsetX->GetMax())/2) ;
            ptrOffsetX->SetValue( ptrOffsetX->GetMin()) ;
            cout << "Offset X set to " << ptrOffsetX->GetMin() << "..." << endl;
        }
        else
        {
            cout << "Offset X not available..." << endl;
        }

        //
        // Apply minimum to offset Y
        //
        // *** NOTES ***
        // It is often desirable to check the increment as well. The increment
        // is a number of which a desired value must be a multiple of. Certain
        // nodes, such as those corresponding to offsets X and Y, have an
        // increment of 1, which basically means that any value within range
        // is appropriate. The increment is retrieved with the method GetInc().
        //
        Spinnaker::GenApi::CIntegerPtr ptrOffsetY = nodeMap.GetNode("OffsetY");
        if (IsAvailable(ptrOffsetY) && IsWritable(ptrOffsetY))
        {
            ptrOffsetY->SetValue(ptrOffsetY->GetMin());
            cout << "Offset Y set to " << ptrOffsetY->GetValue() << "..." << endl;
        }
        else
        {
            cout << "Offset Y not available..." << endl;
        }

        //
        // Set maximum width
        //
        // *** NOTES ***
        // Other nodes, such as those corresponding to image width and height,
        // might have an increment other than 1. In these cases, it can be
        // important to check that the desired value is a multiple of the
        // increment. However, as these values are being set to the maximum,
        // there is no reason to check against the increment.
        //
        Spinnaker::GenApi::CIntegerPtr ptrWidth = nodeMap.GetNode("Width");
        if (IsAvailable(ptrWidth) && IsWritable(ptrWidth))
        {
            int64_t widthToSet = ptrWidth->GetMax();

            ptrWidth->SetValue(widthToSet);

            cout << "Width set to " << ptrWidth->GetValue() << "..." << endl;
        }
        else
        {
            cout << "Width not available..." << endl;
        }

        //
        // Set maximum height
        //
        // *** NOTES ***
        // A maximum is retrieved with the method GetMax(). A node's minimum and
        // maximum should always be a multiple of its increment.
        //
        Spinnaker::GenApi::CIntegerPtr ptrHeight = nodeMap.GetNode("Height");
        if (IsAvailable(ptrHeight) && IsWritable(ptrHeight))
        {
            int64_t heightToSet = ptrHeight->GetMax();

            ptrHeight->SetValue(heightToSet);

            cout << "Height set to " << ptrHeight->GetValue() << "..." << endl << endl;
        }
        else
        {
            cout << "Height not available..." << endl << endl; 
        }


        Spinnaker::GenApi::CIntegerPtr pDeviceLinkThroughputLimit = nodeMap.GetNode("DeviceLinkThroughputLimit");
        if (!IsAvailable(pDeviceLinkThroughputLimit) || !IsWritable(pDeviceLinkThroughputLimit)) {
            cout << "[Device Link Throughput Limit] node not available!!" << endl;
            return -1;
        }
        std::cout << "Device Link Throughput Limit:  " << pDeviceLinkThroughputLimit->GetValue() << std::endl;
        

        pDeviceLinkThroughputLimit->SetValue(824000 + 16000 * 96);
               
        
        std::cout << "Device Link Throughput Limit:  " << pDeviceLinkThroughputLimit->GetValue() << std::endl;
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}

// This function acquires and saves 10 images from a device.
int AcquireImages(CameraPtr pCam, INodeMap& nodeMap, INodeMap& nodeMapTLDevice)
{
    int result = 0;

    cout << endl << endl << "*** IMAGE ACQUISITION ***" << endl << endl;

    try
    {
        //
        // Set acquisition mode to continuous
        //
        // *** NOTES ***
        // Because the example acquires and saves 10 images, setting acquisition
        // mode to continuous lets the example finish. If set to single frame
        // or multiframe (at a lower number of images), the example would just
        // hang. This would happen because the example has been written to
        // acquire 10 images while the camera would have been programmed to
        // retrieve less than that.
        //
        // Setting the value of an enumeration node is slightly more complicated
        // than other node types. Two nodes must be retrieved: first, the
        // enumeration node is retrieved from the nodemap; and second, the entry
        // node is retrieved from the enumeration node. The integer value of the
        // entry node is then set as the new value of the enumeration node.
        //
        // Notice that both the enumeration and the entry nodes are checked for
        // availability and readability/writability. Enumeration nodes are
        // generally readable and writable whereas their entry nodes are only
        // ever readable.
        //
        // Retrieve enumeration node from nodemap
        CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
        if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
        {
            cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << endl << endl;
            return -1;
        }

        // Retrieve entry node from enumeration node
        CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
        if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
        {
            cout << "Unable to set acquisition mode to continuous (entry retrieval). Aborting..." << endl << endl;
            return -1;
        }

        // Retrieve integer value from entry node
        const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

        // Set integer value from entry node as new value of enumeration node
        ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

        cout << "Acquisition mode set to continuous..." << endl;

#ifdef _DEBUG
        cout << endl << endl << "*** DEBUG ***" << endl << endl;

        // If using a GEV camera and debugging, should disable heartbeat first to prevent further issues
        if (DisableHeartbeat(nodeMap, nodeMapTLDevice) != 0)
        {
            return -1;
        }

        cout << endl << endl << "*** END OF DEBUG ***" << endl << endl;
#endif

        //
        // Begin acquiring images
        //
        // *** NOTES ***
        // What happens when the camera begins acquiring images depends on the
        // acquisition mode. Single frame captures only a single image, multi
        // frame captures a set number of images, and continuous captures a
        // continuous stream of images. Because the example calls for the
        // retrieval of 10 images, continuous mode has been set.
        //
        // *** LATER ***
        // Image acquisition must be ended when no more images are needed.
        //
        pCam->BeginAcquisition();

        cout << "Acquiring images..." << endl;

        //
        // Retrieve device serial number for filename
        //
        // *** NOTES ***
        // The device serial number is retrieved in order to keep cameras from
        // overwriting one another. Grabbing image IDs could also accomplish
        // this.
        //
        gcstring deviceSerialNumber("");
        CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
        if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
        {
            deviceSerialNumber = ptrStringSerial->GetValue();

            cout << "Device serial number retrieved as " << deviceSerialNumber << "..." << endl;
        }
        cout << endl;

        // Retrieve, convert, and save images
        const unsigned int k_numImages = 10;

        
        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Publisher pub = it.advertise("stitchData/images", 1);
        ros::Rate loopRate(4);
        while(nh.ok())
        {
            try
            {
                //
                // Retrieve next received image
                //
                // *** NOTES ***
                // Capturing an image houses images on the camera buffer. Trying
                // to capture an image that does not exist will hang the camera.
                //
                // *** LATER ***
                // Once an image from the buffer is saved and/or no longer
                // needed, the image must be released in order to keep the
                // buffer from filling up.
                //
                ImagePtr pResultImage = pCam->GetNextImage(350);

                //
                // Ensure image completion
                //
                // *** NOTES ***
                // Images can easily be checked for completion. This should be
                // done whenever a complete image is expected or required.
                // Further, check image status for a little more insight into
                // why an image is incomplete.
                //
                if (pResultImage->IsIncomplete())
                {
                    // Retrieve and print the image status description
                    cout << "Image incomplete: " << Image::GetImageStatusDescription(pResultImage->GetImageStatus())
                         << "..." << endl
                         << endl;
                }
                else
                {
                    if(!msse) {
                        struct timeval ts;
                        gettimeofday(&ts, NULL);
                        msse = (((unsigned long long) (ts.tv_sec) ) * 1000) + (((unsigned long long) (ts.tv_usec) ) / 1000.0);
                    } else {
                        struct timeval ts;
                        gettimeofday(&ts, NULL);
                        
                        unsigned long long ms = (((unsigned long long) (ts.tv_sec) ) * 1000) + (((unsigned long long) (ts.tv_usec) ) / 1000.0);
                        std::cout << "Last Ms: " << msse << std::endl;
                        std::cout << "Cur  Ms: " << ms << std::endl;
                        std::cout << "Frame Rate: " << (1000.0 / (ms - msse)) << std::endl;
                        msse = ms;
                    }
                    
                    //
                    // Print image information; height and width recorded in pixels
                    //
                    // *** NOTES ***
                    // Images have quite a bit of available metadata including
                    // things such as CRC, image status, and offset values, to
                    // name a few.
                    //
                    const size_t width = pResultImage->GetWidth();

                    const size_t height = pResultImage->GetHeight();

                    cout << "Grabbed image successfully, width = " << width << ", height = " << height << endl;

                    //
                    // Convert image to mono 8
                    //
                    // *** NOTES ***
                    // Images can be converted between pixel formats by using
                    // the appropriate enumeration value. Unlike the original
                    // image, the converted one does not need to be released as
                    // it does not affect the camera buffer.
                    //
                    // When converting images, color processing algorithm is an
                    // optional parameter.
                    //
                    
                    Spinnaker::ImagePtr convertedImage = pResultImage->Convert(Spinnaker::PixelFormat_BGR8, Spinnaker::NEAREST_NEIGHBOR);
                    cv::Mat img = cv::Mat(convertedImage->GetHeight() + convertedImage->GetYPadding(), convertedImage->GetWidth() + convertedImage->GetXPadding(), CV_8UC3, convertedImage->GetData(), convertedImage->GetStride());
                    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
                    pub.publish(msg);
                    ros::spinOnce();
                    loopRate.sleep();
                }

                //
                // Release image
                //
                // *** NOTES ***
                // Images retrieved directly from the camera (i.e. non-converted
                // images) need to be released in order to keep from filling the
                // buffer.
                //
                pResultImage->Release();

                cout << "Successful Release: " << endl;
            }
            catch (Spinnaker::Exception& e) {
                cout << "Acquiisition Error: " << e.what() << endl;
                result = -1;
            }
        }

        //
        // End acquisition
        //
        // *** NOTES ***
        // Ending acquisition appropriately helps ensure that devices clean up
        // properly and do not need to be power-cycled to maintain integrity.
        //

        pCam->EndAcquisition();
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }

    return result;
}

// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int PrintDeviceInfo(INodeMap& nodeMap)
{
    int result = 0;
    cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;

    try
    {
        FeatureList_t features;
        const CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
        if (IsAvailable(category) && IsReadable(category))
        {
            category->GetFeatures(features);

            for (auto it = features.begin(); it != features.end(); ++it)
            {
                const CNodePtr pfeatureNode = *it;
                cout << pfeatureNode->GetName() << " : ";
                CValuePtr pValue = static_cast<CValuePtr>(pfeatureNode);
                cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
                cout << endl;
            }
        }
        else
        {
            cout << "Device control information not available." << endl;
        }
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}

// This function acts as the body of the example; please see NodeMapInfo example
// for more in-depth comments on setting up cameras.
int RunSingleCamera(CameraPtr pCam)
{
    int result;

    try
    {
        // Retrieve TL device nodemap and print device information
        INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

        result = PrintDeviceInfo(nodeMapTLDevice);

        // Initialize camera
        pCam->Init();

        // Retrieve GenICam nodemap
        INodeMap& nodeMap = pCam->GetNodeMap();

        if (ConfigureCustomImageSettings(nodeMap) < 0) {
            std::cerr << "Error Configuring Camera" << std::endl;
            return -1;
        }

        // Acquire images
        result = result | AcquireImages(pCam, nodeMap, nodeMapTLDevice);

        // Deinitialize camera
        pCam->DeInit();
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}


std::ofstream logFile;
void initLog() {
    logFile.open("Image Log.txt", std::ofstream::out);
    logFile.close();
}

void log(std::string msg) {
    logFile.open("Image Log.txt", std::ofstream::app);
    logFile << msg << std::endl;
    logFile.close();
}

// Example entry point; please see Enumeration example for more in-depth
// comments on preparing and cleaning up the system.
int main(int argc, char* argv[])
{
    initLog();

    log("Img Initiated\n");
    

    ros::init(argc, argv, "pub_img");

    log("Ros Inited\n");
    // Since this application saves images in the current folder
    // we must ensure that we have permission to write to this folder.
    // If we do not have permission, fail right away.
    FILE* tempFile = fopen("test.txt", "w+");
    if (tempFile == nullptr)
    {
        cout << "Failed to create file in current folder.  Please check "
                "permissions."
             << endl;
        cout << "Press Enter to exit..." << endl;
        getchar();
        return -1;
    }
    fclose(tempFile);
    remove("test.txt");

    // Print application build information
    cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;

    // Retrieve singleton reference to system object
    SystemPtr system = System::GetInstance();

    // Print out current library version
    const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
    cout << "Spinnaker library version: " << spinnakerLibraryVersion.major << "." << spinnakerLibraryVersion.minor
         << "." << spinnakerLibraryVersion.type << "." << spinnakerLibraryVersion.build << endl
         << endl;

    // Retrieve list of cameras from the system
    CameraList camList = system->GetCameras();

    const unsigned int numCameras = camList.GetSize();

    cout << "Number of cameras detected: " << numCameras << endl << endl;
    log("Cameras Detected\n");
    // Finish if there are no cameras
    if (numCameras == 0)
    {
        // Clear camera list before releasing system
        camList.Clear();

        // Release system
        system->ReleaseInstance();

        cout << "Not enough cameras!" << endl;
        cout << "Done! Press Enter to exit..." << endl;
        getchar();

        return -1;
    }

    //
    // Create shared pointer to camera
    //
    // *** NOTES ***
    // The CameraPtr object is a shared pointer, and will generally clean itself
    // up upon exiting its scope. However, if a shared pointer is created in the
    // same scope that a system object is explicitly released (i.e. this scope),
    // the reference to the shared point must be broken manually.
    //
    // *** LATER ***
    // Shared pointers can be terminated manually by assigning them to nullptr.
    // This keeps releasing the system from throwing an exception.
    //
    CameraPtr pCam = nullptr;

    int result = 0;

    // Run example on each camera
    
    // Select camera
    pCam = camList.GetByIndex(0);

    cout << endl << "Running example for camera 0..." << endl;

    // Run example
    result = result | RunSingleCamera(pCam);

    cout << "Camera 0 example complete..." << endl << endl;
    

    //
    // Release reference to the camera
    //
    // *** NOTES ***
    // Had the CameraPtr object been created within the for-loop, it would not
    // be necessary to manually break the reference because the shared pointer
    // would have automatically cleaned itself up upon exiting the loop.
    //
    pCam = nullptr;

    // Clear camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();

    cout << endl << "Done! Press Enter to exit..." << endl;
    getchar();

    return result;
}
