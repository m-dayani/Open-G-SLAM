//
// Created by root on 5/17/21.
//

#include "ImageHook.h"

namespace OG_SLAM {

    inline void ImageHookFiltered::dispatch(ImagePtr &pImage) {

        mpImageFilter->filter(pImage);
        mpImageQueue->push(pImage);
    }

} // OG_SLAM