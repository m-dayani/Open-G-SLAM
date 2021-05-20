//
// Created by root on 5/17/21.
//

#ifndef OG_SLAM_IMAGEHOOK_H
#define OG_SLAM_IMAGEHOOK_H

#include <memory>
#include <utility>

#include "ImageFilter.h"


namespace OG_SLAM {

    class ImageHook {
    public:
        explicit ImageHook(ImageQueuePtr pImageQueue) : mpImageQueue(std::move(pImageQueue)) {}
        virtual ~ImageHook() = default;

        virtual void dispatch(ImagePtr& pImage) { mpImageQueue->push(pImage); }

    protected:
        ImageQueuePtr mpImageQueue;
    };

    typedef std::shared_ptr<ImageHook> ImageHookPtr;

    class ImageHookFiltered : public ImageHook {
    public:
        ImageHookFiltered(const ImageQueuePtr& pImageQueue, ImageFilterPtr pImFilter) :
                ImageHook(pImageQueue), mpImageFilter(std::move(pImFilter)) {}

        void dispatch(ImagePtr& pImage) override;

    private:
        ImageFilterPtr mpImageFilter;
    };

} // OG_SLAM


#endif //OG_SLAM_IMAGEHOOK_H
