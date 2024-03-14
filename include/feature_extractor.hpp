/**
*    This file is part of OV²SLAM.
*    
*    Copyright (C) 2020 ONERA
*
*    For more information see <https://github.com/ov2slam/ov2slam>
*
*    OV²SLAM is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    OV²SLAM is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with OV²SLAM.  If not, see <https://www.gnu.org/licenses/>.
*
*    Authors: Maxime Ferrera     <maxime.ferrera at gmail dot com> (ONERA, DTIS - IVA),
*             Alexandre Eudes    <first.last at onera dot fr>      (ONERA, DTIS - IVA),
*             Julien Moras       <first.last at onera dot fr>      (ONERA, DTIS - IVA),
*             Martial Sanfourche <first.last at onera dot fr>      (ONERA, DTIS - IVA)
*/
#pragma once


#include "frame.hpp"

/*
给定im、roi、curkps, 在除curkps位置外，提取新特征点并返回。
看配置，作者在fast时更倾向于 gridFast; 在average和accurate时，更倾向于singleScale
另外这些特征点在被提取出来后，还进行了亚像素定位，以期更高的精度
*/
class FeatureExtractor {

public:
    FeatureExtractor() {};
    FeatureExtractor(size_t nmaxpts, size_t nmaxdist, double dmaxquality, int nfast_th);

    std::vector<cv::Point2f> detectGFTT(const cv::Mat &im, const std::vector<cv::Point2f> &vcurkps,
                                        const cv::Mat &roi, int nbmax=-1) const;

    std::vector<cv::Point2f> detectGridFAST(const cv::Mat &im, const int ncellsize, 
        const std::vector<cv::Point2f> &vcurkps, const cv::Rect &roi);

    std::vector<cv::Mat> describeBRIEF(const cv::Mat &im, const std::vector<cv::Point2f> &vpts) const;
    
    std::vector<cv::Point2f> detectSingleScale(const cv::Mat &im, const int ncellsize, 
            const std::vector<cv::Point2f> &vcurkps, const cv::Rect &roi);

    void setMask(const cv::Mat &im, const std::vector<cv::Point2f> &vpts,  const int dist, cv::Mat &mask) const;

    /*
    构造函数设置的参数
    一张图像按 maxdist 分成 r*c 个cell， maxpts = r*c, mindist  = 0.5* maxdist
    maxquality 用于gftt、singleScale的阈值, minquality=0.5*maxquality
    fast_th 用于fast的阈值
    */
    size_t nmaxpts_, nmaxdist_, nmindist_;
    double dmaxquality_, dminquality_;
    int nfast_th_;

    std::vector<int> vumax_;
};