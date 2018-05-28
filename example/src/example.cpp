/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "detectlane.hpp"

//#include <boost/property_tree/ptree.hpp>
//#include <boost/property_tree/ini_parser.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cstdint>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>


int32_t main(int32_t argc, char **argv) {
    int32_t retCode{0};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("name")) || (0 == commandlineArguments.count("cid")) || (0 == commandlineArguments.count("width")) || (0 == commandlineArguments.count("height")) || (0 == commandlineArguments.count("bpp")) || (0 == commandlineArguments.count("blurkernelsize")) || (0 == commandlineArguments.count("adapthreshkernelsize"))
        || (0 == commandlineArguments.count("adapthreshconst")) || (0 == commandlineArguments.count("cannythreshold")) || (0 == commandlineArguments.count("houghthreshold")) || (0 == commandlineArguments.count("linediff")) || (0 == commandlineArguments.count("onelinediff")) || (0 == commandlineArguments.count("horisontallimit"))
        || (0 == commandlineArguments.count("memthreshold")) || (0 == commandlineArguments.count("lowerlanelimit")) || (0 == commandlineArguments.count("upperlanelimit")) || (0 == commandlineArguments.count("roix")) || (0 == commandlineArguments.count("roiy")) || (0 == commandlineArguments.count("roiwidth")) || (0 == commandlineArguments.count("roiheight")) ) {
        std::cerr << argv[0] << " accesses video data using shared memory provided using the command line parameter --name=." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> --width=<width> --height=<height> --bpp=<bits per pixel> --name=<name for the associated shared memory> [--id=<Identifier in case of multiple video streams>] [--verbose]" << std::endl;
        std::cerr << "         --width:   width of a frame" << std::endl;
        std::cerr << "         --height:  height of a frame" << std::endl;
        std::cerr << "         --bpp:     bits per pixel of a frame (either 8 or 24)" << std::endl;
        std::cerr << "         --name:    name of the shared memory to use" << std::endl;
        std::cerr << "         --blurkernelsize:    Size of the blurkernel -> openCV" << std::endl;
        std::cerr << "         --adapthreshkernelsize: Size of the adaptive threshold kernel size -> openCV" << std::endl;
        std::cerr << "         --adapthreshconst:   Adaptive threshold constants -> openCV" << std::endl;
        std::cerr << "         --cannythreshold:    Canny threshold -> openCV" << std::endl;
        std::cerr << "         --houghthreshold:    Hough threshold -> openCV" << std::endl;
        std::cerr << "         --linediff: How much a line can differ" << std::endl;
        std::cerr << "         --onelinediff: How much a line can differ between the two points" << std::endl;
        std::cerr << "         --horisontallimit: meters on each side we should consider" << std::endl;
        std::cerr << "         --memthreshold" << std::endl;
        std::cerr << "         --lowerlanelimit: blue frame to detect lanes lower level" << std::endl;
        std::cerr << "         --upperlanelimit: blue frame to detect lanes upper level"<< std::endl;
        std::cerr << "         --roix: [0]xx Pixels away from the upper left corner in X"<< std::endl;
        std::cerr << "         --roiy: [1]xx Pixels away from the upper part of picture in Y"<< std::endl;
        std::cerr << "         --roiwidth: [2]xx Pixel width of the captured box in X"<< std::endl; 
        std::cerr << "         --roiheight: [3]xx Pixel height of the captured box in Y"<< std::endl; 
        std::cerr << "         --verbose: when set, the image contained in the shared memory is displayed" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=111 --name=cam0 --width=640 --height=480 --bpp=24" << std::endl;
        std::cerr << std::stoi(commandlineArguments["roi"]) << std::endl;
        retCode = 1;
    }
    else {
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const uint32_t BPP{static_cast<uint32_t>(std::stoi(commandlineArguments["bpp"]))};
        
        const uint16_t blurKernelSize{static_cast<uint16_t>(std::stoi(commandlineArguments["blurkernelsize"]))};
        const uint8_t adapThreshKernelSize{static_cast<uint8_t>(std::stoi(commandlineArguments["adapthreshkernelsize"]))};
        const uint8_t adapThreshConst{static_cast<uint8_t>(std::stoi(commandlineArguments["adapthreshconst"]))};
        const uint16_t cannyThreshold{static_cast<uint16_t>(std::stoi(commandlineArguments["cannythreshold"]))};
        const uint16_t houghThreshold{static_cast<uint16_t>(std::stoi(commandlineArguments["houghthreshold"]))};
        const float lineDiff{static_cast<float>(std::stoi(commandlineArguments["linediff"]))};
        const float OneLineDiff{static_cast<float>(std::stoi(commandlineArguments["onelinediff"]))};
        const float HorisontalLimit{static_cast<float>(std::stoi(commandlineArguments["horisontallimit"]))};
        const double memThreshold{static_cast<double>(std::stoi(commandlineArguments["memthreshold"]))};
        const double lowerLaneLimit{static_cast<double>(std::stoi(commandlineArguments["lowerlanelimit"]))};
        const double upperLaneLimit{static_cast<double>(std::stoi(commandlineArguments["upperlanelimit"]))};
        const uint16_t roiX{static_cast<uint16_t>(std::stoi(commandlineArguments["roix"]))};
        const uint16_t roiY{static_cast<uint16_t>(std::stoi(commandlineArguments["roiy"]))};
        const uint16_t roiWidth{static_cast<uint16_t>(std::stoi(commandlineArguments["roiwidth"]))};
        const uint16_t roiHeight{static_cast<uint16_t>(std::stoi(commandlineArguments["roiheight"]))};
        

        if ( (BPP != 24) && (BPP != 8) ) {
            std::cerr << argv[0] << ": bits per pixel must be either 24 or 8; found " << BPP << "." << std::endl;
        }
        else {
            const uint32_t SIZE{WIDTH * HEIGHT * BPP/8};
            const std::string NAME{(commandlineArguments["name"].size() != 0) ? commandlineArguments["name"] : "/cam0"};
            const uint32_t ID{(commandlineArguments["id"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};
            const bool VERBOSE{commandlineArguments.count("verbose") != 0};

            (void)ID;
            (void)SIZE;
            
            //confi-file
            //boost::property_tree::ptree pt;
            //boost::property_tree::ini_parser::read_ini("config.ini", pt);
            
            DetectLane detectlane;

            // Interface to a running OpenDaVINCI session (ignoring any incoming Envelopes).
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            std::unique_ptr<cluon::SharedMemory> sharedMemory(new cluon::SharedMemory{NAME});
            if (sharedMemory && sharedMemory->valid()) {
                std::clog << argv[0] << ": Found shared memory '" << sharedMemory->name() << "' (" << sharedMemory->size() << " bytes)." << std::endl;

                CvSize size;
                size.width = WIDTH;
                size.height = HEIGHT;

                IplImage *image = cvCreateImageHeader(size, IPL_DEPTH_8U, BPP/8);
                sharedMemory->lock();
                image->imageData = sharedMemory->data();
                image->imageDataOrigin = image->imageData;
                sharedMemory->unlock();
                cv::Mat cv_image,cv_image_colorflip;

                while (od4.isRunning()) {
                    // The shared memory uses a pthread broadcast to notify us; just sleep to get awaken up.
                    sharedMemory->wait();
                    sharedMemory->lock();
                    if (VERBOSE) {
                        cvShowImage(sharedMemory->name().c_str(), image);
                        cv_image = cv::cvarrToMat(image);
						
					}
                    sharedMemory->unlock();
                    cv::waitKey(1);
                    
                    cvtColor(cv_image,cv_image_colorflip,cv::COLOR_RGB2BGR);
                    detectlane.Datatrigger(cv_image_colorflip, WIDTH, HEIGHT, blurKernelSize, adapThreshKernelSize, adapThreshConst, cannyThreshold, houghThreshold, lineDiff, OneLineDiff, HorisontalLimit, memThreshold, lowerLaneLimit, upperLaneLimit, roiX, roiY, roiWidth, roiHeight);
                }
                cvReleaseImageHeader(&image);
            }
            else {
                std::cerr << argv[0] << ": Failed to access shared memory '" << NAME << "'." << std::endl;
            }
        }
    }
    return retCode;
}
