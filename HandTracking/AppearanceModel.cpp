//#include <torch/script.h> // One-stop header.
//#include "opencv2/core/cvstd_wrapper.hpp"
//#include "opencv2/tracking.hpp"
//#include "opencv2/videoio.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv2/core.hpp"
//#include <opencv2/objdetect.hpp>
//#include <iostream>
//#include <memory>
//
//using namespace std;
//
////class AppearanceModel
////{
////    std::tuple<at::Tensor, at::Tensor, at::Tensor> attention;
////    torch::jit::script::Module fanet;
////    int window_size = 40;
////    float margin_percent = 0.2;
////    const int patch_size = 224;
////public:
////    AppearanceModel()
////    {
////        try {
////            // Deserialize the ScriptModule from a file using torch::jit::load().
////            fanet = torch::jit::load("D:\\TAU\\Research\\Checkpoints\\traced_appearance_model.pt");
////        }
////        catch (const c10::Error& e) {
////            cerr << "error loading the model\n";
////        }
////
////
////        auto zeros = at::zeros({ 1, });
////        attention = tuple<at::Tensor, at::Tensor, at::Tensor>(zeros, zeros, zeros);
////    }
////
////    vector<cv::Point2f> forward(cv::Mat img, vector<cv::Point2f> prev_keypoints)
////    {
////        using namespace torch::indexing;
////
////#pragma region crop and resize
////        //crop and resize
////
////        cv::Rect myROI = AppearanceModel::calculateROI(prev_keypoints, cv::Size(img.cols, img.rows), margin_percent);
////
////        vector<cv::Point2f> croppedResizedKeypoints;
////        cv::Point2f ratio((float)patch_size / myROI.width, (float)patch_size / myROI.height);
////        for (size_t i = 0; i < prev_keypoints.size(); i++)
////        {
////            auto cropped = prev_keypoints[i] - cv::Point2f(myROI.x, myROI.y);
////            croppedResizedKeypoints.push_back(cv::Point2f(cropped.x*ratio.x, cropped.y*ratio.y));
////        }
////        // Crop the full image to that image contained by the rectangle myROI
////        // Note that this doesn't copy the data
////        cv::Mat croppedImage = img(myROI);
////
////        cv::Mat croppedResizedImage;
////        cv::resize(croppedImage, croppedResizedImage, cv::Size(patch_size, patch_size), 0, 0, CV_INTER_LINEAR);
////
////#pragma endregion
////
////        auto patches = AppearanceModel::normalizedPatches(croppedResizedKeypoints, AppearanceModel::ToTensor(croppedResizedImage), window_size);
////
////        std::vector<torch::jit::IValue> input_to_net = { patches , attention };
////        auto outputs = fanet.forward(input_to_net).toTuple();
////
////        auto coords = outputs->elements()[0].toTensor();
////        //auto heatmaps = outputs->elements()[1].toTensor();
////        auto _new_attention = outputs->elements()[2].toTuple();
////        auto new_attention = tuple<at::Tensor, at::Tensor, at::Tensor>(_new_attention->elements()[0].toTensor(),
////            _new_attention->elements()[1].toTensor(),
////            _new_attention->elements()[2].toTensor());
////
////
////        //(out * self.kp_win_size) - self.kp_win_size / 2 + cuda_noised_labels[:, j]
////        vector<cv::Point2f> regressed_coordinates;
////        auto temp = ((coords * window_size) - window_size / 2);
////        float x, y;
////        for (size_t i = 0; i < prev_keypoints.size(); i++)
////        {
////            x = temp.index({ 0,(int)i,0 }).item().toFloat();
////            y = temp.index({ 0,(int)i,1 }).item().toFloat();
////            regressed_coordinates.push_back(cv::Point2f(prev_keypoints[i].x + x, prev_keypoints[i].y + y));
////        }
////
////        //update attentions
////        attention = new_attention;
////
////        //return regressed coordinates
////        return regressed_coordinates;
////    }
////
////    static at::Tensor ToTensor(cv::Mat img, bool show_output = false, bool unsqueeze = false, int unsqueeze_dim = 0)
////    {
////        at::Tensor tensor_image = torch::from_blob(img.data, { img.rows, img.cols, 3 }, at::kByte);
////        return tensor_image.permute({ 2,0,1 });;
////    }
////
////    static cv::Rect calculateROI(vector<cv::Point2f> keypoints, cv::Size imSize, float margin_prcnt = 0.2)
////    {
////        auto max_x = max_element(keypoints.begin(), keypoints.end(), [](const cv::Point2i &a, const cv::Point2i &b)
////        {
////            return a.x < b.x;
////        });
////        int max_x_ = (*max_x).x;
////
////        auto min_x = min_element(keypoints.begin(), keypoints.end(), [](const cv::Point2i &a, const cv::Point2i &b)
////        {
////            return a.x < b.x;
////        });
////        int min_x_ = (*min_x).x;
////
////
////        auto max_y = max_element(keypoints.begin(), keypoints.end(), [](const cv::Point2i &a, const cv::Point2i &b)
////        {
////            return a.y < b.y;
////        });
////        int max_y_ = (*max_x).y;
////
////        auto min_y = min_element(keypoints.begin(), keypoints.end(), [](const cv::Point2i &a, const cv::Point2i &b)
////        {
////            return a.y < b.y;
////        });
////        int min_y_ = (*min_y).y;
////
////        int width = max_x_ - min_x_;
////        int height = max_y_ - min_y_;
////
////        int img_width = imSize.width;
////        int img_height = imSize.height;
////
////        min_x_ = min(max((int)(min_x_ - width * margin_prcnt), 0), img_width);
////        max_x_ = min(max((int)(max_x_ + width * margin_prcnt + 1), 0), img_width);
////        min_y_ = min(max((int)(min_y_ - height * margin_prcnt), 0), img_height);
////        max_y_ = min(max((int)(max_y_ + height * margin_prcnt + 1), 0), img_height);
////
////        return cv::Rect(min_x_, min_y_, max_x_ - min_x_, max_y_ - min_y_);
////    }
////
////    static at::Tensor normalizedPatches(vector<cv::Point2f> keypoints, at::Tensor imgTensor, int window_size = 40)
////    {
////        using namespace torch::indexing;
////        int top, left;
////        auto circle_window = at::ones({ window_size,window_size }, at::kFloat);
////
////        auto patches = at::zeros({ 1,(long long)keypoints.size(),window_size,window_size }, at::kFloat);
////        for (size_t i = 0; i < keypoints.size(); i++)
////        {
////            top = (int)keypoints[i].y - window_size / 2;
////            left = (int)keypoints[i].x - window_size / 2;
////            auto patch = imgTensor.index({ 0, Slice(top,top + window_size),Slice(left,left + window_size) }).toType(at::kFloat);
////            patch -= patch.mean();
////            patch /= patch.std();
////            patch *= circle_window;
////            patches.index_put_({ 0, (int)i, Slice(), Slice() }, patch);
////        }
////        return patches;
////    }
////
////};
//
//
//
//std::string get_image_type(const cv::Mat& img, bool more_info = true)
//{
//    std::string r;
//    int type = img.type();
//    uchar depth = type & CV_MAT_DEPTH_MASK;
//    uchar chans = 1 + (type >> CV_CN_SHIFT);
//
//    switch (depth) {
//    case CV_8U:  r = "8U"; break;
//    case CV_8S:  r = "8S"; break;
//    case CV_16U: r = "16U"; break;
//    case CV_16S: r = "16S"; break;
//    case CV_32S: r = "32S"; break;
//    case CV_32F: r = "32F"; break;
//    case CV_64F: r = "64F"; break;
//    default:     r = "User"; break;
//    }
//
//    r += "C";
//    r += (chans + '0');
//
//    if (more_info)
//        std::cout << "depth: " << img.depth() << " channels: " << img.channels() << std::endl;
//
//    return r;
//}
//
//void show_image(cv::Mat& img, std::string title)
//{
//    std::string image_type = get_image_type(img);
//    cv::imshow(title, img);
//    cv::waitKey(0);
//}
//
//auto transpose(at::Tensor tensor, c10::IntArrayRef dims = { 0, 3, 1, 2 })
//{
//    std::cout << "############### transpose ############" << std::endl;
//    std::cout << "shape before : " << tensor.sizes() << std::endl;
//    tensor = tensor.permute(dims);
//    std::cout << "shape after : " << tensor.sizes() << std::endl;
//    std::cout << "######################################" << std::endl;
//    return tensor;
//}
//
//auto ToTensor(cv::Mat img, bool show_output = false, bool unsqueeze = false, int unsqueeze_dim = 0)
//{
//    at::Tensor tensor_image = torch::from_blob(img.data, { img.rows, img.cols, 3 }, at::kByte);
//    return tensor_image.permute({ 2,0,1 });;
//}
//
//auto ToInput(at::Tensor tensor_image)
//{
//    // Create a vector of inputs.
//    return std::vector<torch::jit::IValue>{tensor_image};
//}
//
//auto ToCvImage(at::Tensor tensor)
//{
//    int width = tensor.sizes()[0];
//    int height = tensor.sizes()[1];
//    try
//    {
//        cv::Mat output_mat(cv::Size{ height, width }, CV_8UC3, tensor.data_ptr<uchar>());
//
//        show_image(output_mat, "converted image from tensor");
//        return output_mat.clone();
//    }
//    catch (const c10::Error& e)
//    {
//        std::cout << "an error has occured : " << e.msg() << std::endl;
//    }
//    return cv::Mat(height, width, CV_8UC3);
//}
//
//
//cv::Rect calculateROI(vector<cv::Point2f> keypoints, cv::Size imSize, float margin_prcnt = 0.2)
//{
//    auto max_x = max_element(keypoints.begin(), keypoints.end(), [](const cv::Point2i &a, const cv::Point2i &b)
//    {
//        return a.x < b.x;
//    });
//    int max_x_ = (*max_x).x;
//
//    auto min_x = min_element(keypoints.begin(), keypoints.end(), [](const cv::Point2i &a, const cv::Point2i &b)
//    {
//        return a.x < b.x;
//    });
//    int min_x_ = (*min_x).x;
//
//
//    auto max_y = max_element(keypoints.begin(), keypoints.end(), [](const cv::Point2i &a, const cv::Point2i &b)
//    {
//        return a.y < b.y;
//    });
//    int max_y_ = (*max_x).y;
//
//    auto min_y = min_element(keypoints.begin(), keypoints.end(), [](const cv::Point2i &a, const cv::Point2i &b)
//    {
//        return a.y < b.y;
//    });
//    int min_y_ = (*min_y).y;
//
//    int width = max_x_ - min_x_;
//    int height = max_y_ - min_y_;
//
//    int img_width = imSize.width;
//    int img_height = imSize.height;
//
//    min_x_ = min(max((int)(min_x_ - width * margin_prcnt), 0), img_width);
//    max_x_ = min(max((int)(max_x_ + width * margin_prcnt + 1), 0), img_width);
//    min_y_ = min(max((int)(min_y_ - height * margin_prcnt), 0), img_height);
//    max_y_ = min(max((int)(max_y_ + height * margin_prcnt + 1), 0), img_height);
//
//    return cv::Rect(min_x_, min_y_, max_x_ - min_x_, max_y_ - min_y_);
//}
//
//at::Tensor normalizedPatches(vector<cv::Point2f> keypoints, at::Tensor imgTensor, int window_size = 40)
//{
//    using namespace torch::indexing;
//    int top, left;
//    auto circle_window = at::ones({ window_size,window_size }, at::kFloat);
//
//    auto patches = at::zeros({ 1,(long long)keypoints.size(),window_size,window_size }, at::kFloat);
//    for (size_t i = 0; i < keypoints.size(); i++)
//    {
//        top = (int)keypoints[i].y - window_size / 2;
//        left = (int)keypoints[i].x - window_size / 2;
//        auto patch = imgTensor.index({ 0, Slice(top,top + window_size),Slice(left,left + window_size) }).toType(at::kFloat);
//        patch -= patch.mean();
//        patch /= patch.std();
//        patch *= circle_window;
//        patches.index_put_({ 0, (int)i, Slice(), Slice() }, patch);
//    }
//    return patches;
//}
//
//
//// the appearance model network output. regressed coordinates and attention.
//tuple<vector<cv::Point2f>, tuple<at::Tensor, at::Tensor, at::Tensor>> AppearanceModel(cv::Mat img, vector<cv::Point2f> prev_keypoints, torch::jit::script::Module fanet, tuple<at::Tensor, at::Tensor, at::Tensor> attention)
//{
//    using namespace torch::indexing;
//    int window_size = 40;
//
//#pragma region crop and resize
//    //crop and resize
//
//    cv::Rect myROI = calculateROI(prev_keypoints, cv::Size(img.cols, img.rows));
//
//    vector<cv::Point2f> croppedResizedKeypoints;
//    cv::Point2f ratio(224.0 / myROI.width, 224.0 / myROI.height);
//    for (size_t i = 0; i < prev_keypoints.size(); i++)
//    {
//        auto cropped = prev_keypoints[i] - cv::Point2f(myROI.x, myROI.y);
//        croppedResizedKeypoints.push_back(cv::Point2f(cropped.x*ratio.x, cropped.y*ratio.y));
//    }
//    // Crop the full image to that image contained by the rectangle myROI
//    // Note that this doesn't copy the data
//    cv::Mat croppedImage = img(myROI);
//
//    cv::Mat croppedResizedImage;
//    cv::resize(croppedImage, croppedResizedImage, cv::Size(224, 224), 0, 0, CV_INTER_LINEAR);
//
//#pragma endregion
//
//    auto patches = normalizedPatches(croppedResizedKeypoints, ToTensor(croppedResizedImage), window_size);
//
//    std::vector<torch::jit::IValue> input_to_net = { patches , attention };
//    auto outputs = fanet.forward(input_to_net).toTuple();
//
//    auto coords = outputs->elements()[0].toTensor();
//    //auto heatmaps = outputs->elements()[1].toTensor();
//    auto _new_attention = outputs->elements()[2].toTuple();
//    auto new_attention = tuple<at::Tensor, at::Tensor, at::Tensor>(_new_attention->elements()[0].toTensor(),
//        _new_attention->elements()[1].toTensor(),
//        _new_attention->elements()[2].toTensor());
//
//    //(out * self.kp_win_size) - self.kp_win_size / 2 + cuda_noised_labels[:, j]
//    vector<cv::Point2f> regressed_coordinates;
//    auto temp = ((coords * window_size) - window_size / 2);
//    float x, y;
//    for (size_t i = 0; i < prev_keypoints.size(); i++)
//    {
//        x = temp.index({ 0,(int)i,0 }).item().toFloat();
//        y = temp.index({ 0,(int)i,1 }).item().toFloat();
//        regressed_coordinates.push_back(cv::Point2f(prev_keypoints[i].x + x, prev_keypoints[i].y + y));
//    }
//    return tuple<vector<cv::Point2f>, tuple<at::Tensor, at::Tensor, at::Tensor>>(regressed_coordinates, new_attention);
//}
//
//int main6778(int argc, const char* argv[]) {
//
//
//    torch::jit::script::Module fanet;
//    try {
//        // Deserialize the ScriptModule from a file using torch::jit::load().
//        fanet = torch::jit::load("D:\\TAU\\Research\\Checkpoints\\traced_appearance_model.pt");
//        //cout << module.dump_to_str(true, false, false, 1);
//
//    }
//    catch (const c10::Error& e) {
//        cerr << "error loading the model\n";
//        return -1;
//    }
//
//
//    //inputs
//    std::string msg = "sample image";
//    auto currentPath = "C:\\Users\\ofir\\Desktop\\dad.png";
//    auto img = cv::imread(currentPath);
//    show_image(img, msg);
//
//    vector<cv::Point2f> keypoints = { cv::Point2f(522, 372),
//                                    cv::Point2f(506, 328),
//                                    cv::Point2f(482, 283),
//                                    cv::Point2f(452, 257),
//                                    cv::Point2f(432, 240),
//                                    cv::Point2f(425, 301),
//                                    cv::Point2f(382, 272),
//                                    cv::Point2f(359, 250),
//                                    cv::Point2f(343, 233),
//                                    cv::Point2f(406, 325),
//                                    cv::Point2f(357, 296),
//                                    cv::Point2f(330, 274),
//                                    cv::Point2f(309, 258),
//                                    cv::Point2f(396, 345),
//                                    cv::Point2f(350, 322),
//                                    cv::Point2f(324, 304),
//                                    cv::Point2f(306, 288),
//                                    cv::Point2f(393, 365),
//                                    cv::Point2f(354, 351),
//                                    cv::Point2f(333, 339),
//                                    cv::Point2f(317, 326) };
//
//    std::cout << keypoints << std::endl;
//
//    auto zeros = at::zeros({ 1, });
//    auto attentions = tuple<at::Tensor, at::Tensor, at::Tensor>(zeros, zeros, zeros);
//    auto result = AppearanceModel(img, keypoints, fanet, attentions);
//    std::cout << std::get<0>(result) << std::endl;
//
//    return 0;
//
//}
//
//
////
////#pragma endregion
////
////    //crop and resize
////#pragma region crop and resize
////
////
////    cv::Rect myROI = calculateROI(keypoints, cv::Size(img.cols, img.rows));
////
////    vector<cv::Point2f> croppedResizedKeypoints;
////    cv::Point2f ratio(224.0 / myROI.width, 224.0 / myROI.height);
////    for (size_t i = 0; i < keypoints.size(); i++)
////    {
////        auto cropped = keypoints[i] - cv::Point2f(myROI.x, myROI.y);
////        croppedResizedKeypoints.push_back(cv::Point2f(cropped.x*ratio.x, cropped.y*ratio.y));
////    }
////    // Crop the full image to that image contained by the rectangle myROI
////    // Note that this doesn't copy the data
////    cv::Mat croppedImage = img(myROI);
////
////    cv::Mat croppedResizedImage;
////    cv::resize(croppedImage, croppedResizedImage, cv::Size(224, 224), 0, 0, CV_INTER_LINEAR);
////
////
////#pragma endregion
////
////
////#pragma region Patches extraction, to tensor and normalization
////
////    auto patches = normalizedPatches(croppedResizedKeypoints, ToTensor(croppedResizedImage));
////#pragma endregion
////
////
////    auto zero = at::zeros({ 1, });
////    std::vector<torch::jit::IValue> input_to_net = { patches , tuple<at::Tensor,at::Tensor,at::Tensor>(zero,zero,zero) };
////    auto outputs = fanet.forward(input_to_net).toTuple();
////
////    auto coords = outputs->elements()[0].toTensor();
////    auto heatmaps = outputs->elements()[1].toTensor();
////    auto attention = outputs->elements()[2].toTuple();
////    // Execute the model and turn its output into a tensor.
////    std::cout << coords.sizes() << std::endl;
////    std::cout << "output: " << coords << std::endl;
////    std::cout << "ok\n";
////}
//////522.0, 372.0, 506.0, 328.0, 482.0, 283.0, 452.0, 257.0, 432.0, 240.0, 425.0, 301.0, 382.0, 272.0, 359.0, 250.0, 343.0, 233.0, 406.0, 325.0, 357.0, 296.0, 330.0, 274.0, 309.0, 258.0, 396.0, 345.0, 350.0, 322.0, 324.0, 304.0, 306.0, 288.0, 393.0, 365.0, 354.0, 351.0, 333.0, 339.0, 317.0, 326.0