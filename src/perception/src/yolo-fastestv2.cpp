#include <math.h>
#include <algorithm>
#include "yolo-fastestv2.h"

//模型的参数配置
yoloFastestv2::yoloFastestv2()
{   
    printf("Create yoloFastestv2 Detector...\n");
    //输出节点数
    numOutput = 2;
    //推理线程数
    numThreads = 6;
    //anchor num
    numAnchor = 3;
    //类别数目
    numCategory = 13;
    //NMS阈值
    nmsThresh = 0.15;

    //模型输入尺寸大小
    inputWidth = 352;
    inputHeight = 352;

    //模型输入输出节点名称
    inputName = "input.1";
    outputName1 = "794"; //22x22 #794
    outputName2 = "796"; //11x11 #796

    //打印初始化相关信息
    printf("numThreads:%d\n", numThreads);
    printf("inputWidth:%d inputHeight:%d\n", inputWidth, inputHeight);

    //anchor box w h
    std::vector<float> bias {16.89,26.94, 29.89,76.88, 33.68,41.00, 55.10,129.28, 85.09,57.66, 138.09,99.07};

    anchor.assign(bias.begin(), bias.end());
}

yoloFastestv2::~yoloFastestv2()
{
    printf("Destroy yoloFastestv2 Detector...\n");
}

//ncnn 模型加载
int yoloFastestv2::loadModel(const char* paramPath, const char* binPath)
{
    printf("Ncnn mode init:\n%s\n%s\n", paramPath, binPath);

    net.load_param(paramPath);
    net.load_model(binPath);

    printf("Ncnn model init sucess...\n");

    return 0;
}

float intersection_area(const TargetBox &a, const TargetBox &b)
{
    // std::cout << "intersection_area" << std::endl;
    if (a.x1 > b.x2 || a.x2 < b.x1 || a.y1 > b.y2 || a.y2 < b.y1)
    {
        // no intersection
        return 0.f;
    }

    float inter_width = std::min(a.x2, b.x2) - std::max(a.x1, b.x1);
    float inter_height = std::min(a.y2, b.y2) - std::max(a.y1, b.y1);

    // std::cout << "end intersection_area" << std::endl;
    return inter_width * inter_height;
}

bool scoreSort(TargetBox a, TargetBox b) 
{ 
    return (a.score > b.score); 
}

//NMS处理
int yoloFastestv2::nmsHandle(std::vector<TargetBox> &tmpBoxes, 
                             std::vector<TargetBox> &dstBoxes)
{
    // std::cout << "nmsHandle" << std::endl;
    std::vector<int> picked;
    
    sort(tmpBoxes.begin(), tmpBoxes.end(), scoreSort);

    for (int i = 0; i < tmpBoxes.size(); i++) {
        int keep = 1;
        for (int j = 0; j < picked.size(); j++) {
            //交集
            float inter_area = intersection_area(tmpBoxes[i], tmpBoxes[picked[j]]);
            //并集
            float union_area = tmpBoxes[i].area() + tmpBoxes[picked[j]].area() - inter_area;
            float IoU = inter_area / union_area;

            if(IoU > nmsThresh && tmpBoxes[i].cate == tmpBoxes[picked[j]].cate) {
                keep = 0;
                break;
            }
        }

        if (keep) {
            picked.push_back(i);
        }
    }
    
    for (int i = 0; i < picked.size(); i++) {
        dstBoxes.push_back(tmpBoxes[picked[i]]);
    }
    // std::cout << "end nmsHandle" << std::endl;
    return 0;
}

//检测类别分数处理
int yoloFastestv2::getCategory(const float *values, int index, int &category, float &score)
{
    // std::cout << "getCategory" << std::endl;
    float tmp = 0;
    float objScore  = values[4 * numAnchor + index];

    for (int i = 0; i < numCategory; i++) {
        float clsScore = values[4 * numAnchor + numAnchor + i];
        clsScore *= objScore;

        if(clsScore > tmp) {
            score = clsScore;
            category = i;

            tmp = clsScore;
        }
    }
    
    // std::cout << "end getCategory" << std::endl;
    return 0;
}

//特征图后处理
int yoloFastestv2::predHandle(const ncnn::Mat *out, std::vector<TargetBox> &dstBoxes, 
                              const float scaleW, const float scaleH, const float thresh)
{    
    // std::cout << "predHandle" << std::endl;
    //do result
    for (int i = 0; i < numOutput; i++) {   
        int stride;
        int outW, outH, outC;

        outH = out[i].c;
        outW = out[i].h;
        outC = out[i].w;
        // std::cout << "hi1" << std::endl;
        // std::cout << "outH:" << outH << " outW:" << outW << " outC:" << outC << std::endl;
        assert(inputHeight / outH == inputWidth / outW);
        stride = inputHeight / outH;

        for (int h = 0; h < outH; h++) {
            const float* values = out[i].channel(h);

            for (int w = 0; w < outW; w++) {
                for (int b = 0; b < numAnchor; b++) {                    
                    //float objScore = values[4 * numAnchor + b];
                    TargetBox tmpBox;
                    int category = -1;
                    float score = -1;

                    getCategory(values, b, category, score);

                    if (score > thresh) {
                        float bcx, bcy, bw, bh;

                        bcx = ((values[b * 4 + 0] * 2. - 0.5) + w) * stride;
                        bcy = ((values[b * 4 + 1] * 2. - 0.5) + h) * stride;
                        bw = pow((values[b * 4 + 2] * 2.), 2) * anchor[(i * numAnchor * 2) + b * 2 + 0];
                        bh = pow((values[b * 4 + 3] * 2.), 2) * anchor[(i * numAnchor * 2) + b * 2 + 1];
                        
                        tmpBox.x1 = (bcx - 0.5 * bw) * scaleW;
                        tmpBox.y1 = (bcy - 0.5 * bh) * scaleH;
                        tmpBox.x2 = (bcx + 0.5 * bw) * scaleW;
                        tmpBox.y2 = (bcy + 0.5 * bh) * scaleH;
                        tmpBox.score = score;
                        tmpBox.cate = category;

                        dstBoxes.push_back(tmpBox);
                    }
                }
                values += outC;
            } 
        } 
    }
    // std::cout << "end predHandle" << std::endl;
    return 0;
}

int yoloFastestv2::detection(const cv::Mat srcImg, std::vector<TargetBox> &dstBoxes, const float thresh)
{   
    // std::cout << "detection" << std::endl;
    dstBoxes.clear();

    float scaleW = (float)srcImg.cols / (float)inputWidth;
    float scaleH = (float)srcImg.rows / (float)inputHeight;
    
    //resize of input image data
    ncnn::Mat inputImg = ncnn::Mat::from_pixels_resize(srcImg.data, ncnn::Mat::PIXEL_BGR,\
                                                       srcImg.cols, srcImg.rows, inputWidth, inputHeight); 

    //Normalization of input image data
    const float mean_vals[3] = {0.f, 0.f, 0.f};
    const float norm_vals[3] = {1/255.f, 1/255.f, 1/255.f};
    inputImg.substract_mean_normalize(mean_vals, norm_vals);  

    //creat extractor
    ncnn::Extractor ex = net.create_extractor();
    // ex.set_num_threads(numThreads);

    //set input tensor
    ex.input(inputName, inputImg);

    //forward
    ncnn::Mat out[2]; 
    ex.extract(outputName1, out[0]); //22x22
    ex.extract(outputName2, out[1]); //11x11

    std::vector<TargetBox> tmpBoxes;
    //特征图后处理
    predHandle(out, tmpBoxes, scaleW, scaleH, thresh);

    //NMS
    nmsHandle(tmpBoxes, dstBoxes);
    
    // std::cout << "end detection" << std::endl;
    return 0;
}
