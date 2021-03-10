#pragma once
#include <torch/script.h> // One-stop header.
#include "opencv2/core/cvstd_wrapper.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include <opencv2/objdetect.hpp>
#include <iostream>
#include <memory>

using namespace std;

class AppearanceModel
{
    at::Tensor attention0;
    at::Tensor attention1;
    at::Tensor attention2;
    torch::jit::script::Module fanet;
    int window_size = 40;
    float margin_percent = 0.2;
    const int patch_size = 224;
public:
    AppearanceModel()
    {
        try {
            // Deserialize the ScriptModule from a file using torch::jit::load().
            fanet = torch::jit::load("D:\\TAU\\Research\\Checkpoints\\traced_appearance_model.pt");
        }
        catch (const c10::Error& e) {
            cerr << "error loading the model\n";
        }


        attention0 = at::zeros({ 1, });
        attention1 = at::zeros({ 1, });
        attention2 = at::zeros({ 1, });
    }

    //vector<cv::Point2f> forward(cv::Mat img, vector<cv::Point2f> prev_keypoints)
    void forward(cv::Mat img, vector<cv::Point2f>& prev_keypoints)
    {

        using namespace torch::indexing;

#pragma region crop and resize
        //crop and resize

        cv::Rect myROI = AppearanceModel::calculateROI(prev_keypoints, cv::Size(img.cols, img.rows), margin_percent);


        vector<cv::Point2f> croppedResizedKeypoints;
        cv::Point2f ratio((float)patch_size / myROI.width, (float)patch_size / myROI.height);
        for (size_t i = 0; i < prev_keypoints.size(); i++)
        {
            auto cropped = prev_keypoints[i] - cv::Point2f(myROI.x, myROI.y);
            croppedResizedKeypoints.push_back(cv::Point2f(cropped.x*ratio.x, cropped.y*ratio.y));
        }
        // Crop the full image to that image contained by the rectangle myROI
        // Note that this doesn't copy the data
        cv::Mat croppedImage = img(myROI);

        cv::Mat croppedResizedImage;
        cv::resize(croppedImage, croppedResizedImage, cv::Size(patch_size, patch_size), 0, 0, CV_INTER_LINEAR);


#pragma endregion

        auto patches = AppearanceModel::normalizedPatches(croppedResizedKeypoints, AppearanceModel::ToTensor(croppedResizedImage), window_size);
        auto attention = tuple<at::Tensor, at::Tensor, at::Tensor>(attention0, attention1, attention2);
        std::vector<torch::jit::IValue> input_to_net = { patches , attention };
        auto outputs = fanet.forward(input_to_net).toTuple();

        auto coords = outputs->elements()[0].toTensor();
        //auto heatmaps = outputs->elements()[1].toTensor();
        auto _new_attention = outputs->elements()[2].toTuple();
        
        //update attentions
        attention0.set_data(_new_attention->elements()[0].toTensor());
        attention1.set_data(_new_attention->elements()[1].toTensor());
        attention2.set_data(_new_attention->elements()[2].toTensor());

        //(out * self.kp_win_size) - self.kp_win_size / 2 + cuda_noised_labels[:, j]
        vector<cv::Point2f> regressed_coordinates;
        auto temp = ((coords * window_size) - window_size / 2);
        float x, y;
        for (size_t i = 0; i < prev_keypoints.size(); i++)
        {
            x = temp.index({ 0,(int)i,0 }).item().toFloat();
            y = temp.index({ 0,(int)i,1 }).item().toFloat();
            //regressed_coordinates.push_back(cv::Point2f(prev_keypoints[i].x + x, prev_keypoints[i].y + y));
            prev_keypoints[i].x += x;
            prev_keypoints[i].y += y;
        }

        //return regressed_coordinates;
    }

    static at::Tensor ToTensor(cv::Mat img, bool show_output = false, bool unsqueeze = false, int unsqueeze_dim = 0)
    {
        at::Tensor tensor_image = torch::from_blob(img.data, { img.rows, img.cols, 3 }, at::kByte);
        return tensor_image.permute({ 2,0,1 });
    }

    static cv::Rect calculateROI(vector<cv::Point2f> keypoints, cv::Size imSize, float margin_prcnt = 0.2)
    {
        auto max_x = max_element(keypoints.begin(), keypoints.end(), [](const cv::Point2i &a, const cv::Point2i &b)
        {
            return a.x < b.x;
        });
        int max_x_ = (*max_x).x;

        auto min_x = min_element(keypoints.begin(), keypoints.end(), [](const cv::Point2i &a, const cv::Point2i &b)
        {
            return a.x < b.x;
        });
        int min_x_ = (*min_x).x;


        auto max_y = max_element(keypoints.begin(), keypoints.end(), [](const cv::Point2i &a, const cv::Point2i &b)
        {
            return a.y < b.y;
        });
        int max_y_ = (*max_x).y;

        auto min_y = min_element(keypoints.begin(), keypoints.end(), [](const cv::Point2i &a, const cv::Point2i &b)
        {
            return a.y < b.y;
        });
        int min_y_ = (*min_y).y;

        int width = max_x_ - min_x_;
        int height = max_y_ - min_y_;

        int img_width = imSize.width;
        int img_height = imSize.height;

        min_x_ = min(max((int)(min_x_ - width * margin_prcnt), 0), img_width);
        max_x_ = min(max((int)(max_x_ + width * margin_prcnt + 1), 0), img_width);
        min_y_ = min(max((int)(min_y_ - height * margin_prcnt), 0), img_height);
        max_y_ = min(max((int)(max_y_ + height * margin_prcnt + 1), 0), img_height);

        return cv::Rect(min_x_, min_y_, max_x_ - min_x_, max_y_ - min_y_);
    }

    static at::Tensor normalizedPatches(vector<cv::Point2f> keypoints, at::Tensor imgTensor, int window_size = 40)
    {
        using namespace torch::indexing;
        int top, left;
        auto circle_window = at::ones({ window_size,window_size }, at::kFloat);

        auto patches = at::zeros({ 1,(long long)keypoints.size(),window_size,window_size }, at::kFloat);
        for (size_t i = 0; i < keypoints.size(); i++)
        {
            top = (int)keypoints[i].y - window_size / 2;
            left = (int)keypoints[i].x - window_size / 2;
            auto patch = imgTensor.index({ 0, Slice(top,top + window_size),Slice(left,left + window_size) }).toType(at::kFloat);

            patch -= patch.mean();
            patch /= patch.std() + 1e-3;
            patch *= circle_window.index({ Slice(0,patch.sizes()[0]), Slice(0,patch.sizes()[1]) });
            patches.index_put_({ 0, (int)i, Slice(0,patch.sizes()[0]), Slice(0,patch.sizes()[1]) }, patch);
        }
        return patches;
    }

};
