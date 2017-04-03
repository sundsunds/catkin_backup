#include "libks/imageio/sharedimagesetpublisher.h"

namespace ks {

	using namespace cv;
	using namespace std;
	using namespace ros;

	SharedImageSetPublisher::SharedImageSetPublisher(NodeHandle& nh, const char* topic,
		unsigned int queueSize, unsigned int sharedMemSize):
        sharedMemSize(sharedMemSize), publisher(nh.advertise<libks_msgs::SharedImageSet>(topic, queueSize)) {
	}

    libks_msgs::SharedImageSetConstPtr SharedImageSetPublisher::createMessage(const std_msgs::Header& header,
                                                                         const cv::Mat& img1, const cv::Mat& img2,
                                                                         const cv::Mat& img3, const cv::Mat& img4)
    {
        Mat* imgPtrs[4] = {
            new Mat(img1), new Mat(img2), new Mat(img3), new Mat(img4)
        };
        return createMessage(header, imgPtrs, 4);
    }

    libks_msgs::SharedImageSetConstPtr SharedImageSetPublisher::createMessage(const std_msgs::Header& header, cv::Mat** imgs, unsigned int numImgs)
    {
        libks_msgs::SharedImageSetPtr msg(new libks_msgs::SharedImageSet);
        msg->header = header;

        // Keep pointers alive for a while and then delete
        for(unsigned int i=0; i<numImgs; i++) {
            sharedMemHistory.push(imgs[i]);
            msg->imagePtrs.push_back((unsigned long long) imgs[i]);
        }
        while(sharedMemHistory.size() > 100) {
            delete sharedMemHistory.front();
            sharedMemHistory.pop();
        }

        return msg;
    }

    //raj msg
    libks_msgs::SharedImageSetConstPtr SharedImageSetPublisher::createMessage(const std_msgs::Header& header,
                                                                         const cv::Mat& img1)
    {
        Mat* imgPtrs[1] = {
            new Mat(img1)
        };
        return createMessage(header, imgPtrs, 1);
    }


	void SharedImageSetPublisher::publish(const std_msgs::Header& header, const Mat& img1) {
		Mat* imgPtr = new Mat(img1);
		publish(header, &imgPtr, 1);
	}

	void SharedImageSetPublisher::publish(const std_msgs::Header& header, const Mat& img1,
		const Mat& img2) {
		Mat* imgPtrs[2] = {
			new Mat(img1), new Mat(img2)
		};
		publish(header, imgPtrs, 2);
	}

	void SharedImageSetPublisher::publish(const std_msgs::Header& header, const Mat& img1,
		const Mat& img2, const Mat& img3, const Mat& img4) {
        publish(createMessage(header, img1, img2, img3, img4));
	}

	void SharedImageSetPublisher::publish(const std_msgs::Header& header, cv::Mat** imgs, unsigned int numImgs) {
        publish(createMessage(header, imgs, numImgs));
	}

    void SharedImageSetPublisher::publish(libks_msgs::SharedImageSetConstPtr msg) {
        publisher.publish(msg);
    }
}
