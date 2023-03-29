#include "energy_state.h"
#include <opencv2/core/eigen.hpp>


#define SHOW_BINARY
#define SHOW_FU
#define AIM_SHOW
//#define R_ROI_SHOW
#define R_SHOW
//#define R_RECT_SHOW
//#define SEE_AREA_DIFF

using namespace cv;

energy::energy()
{
    l_hit = imread("../other/l_h.jpg");
    r_hit = imread("../other/r_h.jpg");
    l_uhit = imread("../other/l_uh.jpg");
    r_uhit = imread("../other/r_uh.jpg");
    cvtColor(l_hit,l_hit,COLOR_BGR2GRAY);
    cvtColor(r_hit,r_hit,COLOR_BGR2GRAY);
    cvtColor(l_uhit,l_uhit,COLOR_BGR2GRAY);
    cvtColor(r_uhit,r_uhit,COLOR_BGR2GRAY);
}



energy_inf energy::detect_aim()
{
    hit_count = 0;
    energy_inf energyInf;
    //    src = _src.clone();

    Mat max_color = Mat(src.size(), CV_8UC1, cv::Scalar(0));
    Mat thres_b;
    int width = src.cols;
    int height = src.rows;
    int srcdata = width*height;

    uchar* Imgdata = (uchar*)src.data;
    uchar* Imgdata_binary = (uchar*)max_color.data;
    cvtColor(src,thres_b,COLOR_BGR2GRAY);
    threshold(thres_b,thres_b,thresh,255,THRESH_BINARY);//这里阈值参数要调
    if (this->enemy_color == RED)
    {
        for (int i=0;i<srcdata;i++)
        {
            if (*(Imgdata+2) - *Imgdata > thres_red)*Imgdata_binary = 255;//这里阈值参数也要调
            Imgdata_binary++;
            Imgdata+=3;
        }
    }
    else
    {
        for (int i=0;i<srcdata;i++)
        {
            if (*(Imgdata) - *(Imgdata+2) > thres_blue)*Imgdata_binary = 255;//这里阈值参数也要调
            Imgdata_binary++;
            Imgdata+=3;
        }
    }
    max_color = thres_b & max_color;
    center_mat = max_color.clone();
    Mat dilate_color;
    Mat kernel = getStructuringElement(MORPH_ELLIPSE,Size(3,3));//这个膨胀核也要调
    dilate(max_color,max_color,kernel);
#ifdef SHOW_BINARY
    imshow("binary",max_color);
#endif
    std::vector<std::vector<Point2f>> cnt;
    findContours(max_color,cnt,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
    int recent_hited = 0;
    int min_index = 0;
    Mat result;
    Point2f re_p[4];
    int found = 0;
    for (int i=0;i<cnt.size();i++)
    {
        RotatedRect r = minAreaRect(cnt[i]);
        double r_w = r.size.width;
        double r_h = r.size.height;
        double wh_area = r_w*r_h;
        if ((wh_area> 3000)&&(wh_area<30000))
        {
            double co_area = contourArea(cnt[i]);
#ifdef SEE_AREA_DIFF
            std::cout<<co_area<<std::endl;
            std::cout<<wh_area<<std::endl;
#endif
            //drawContours(src,cnt,i,Scalar(255,0,0),2);
            if((co_area/wh_area < 0.58)&&(co_area/wh_area > 0.3)&&!found)
            {
                //cout<<co_area/wh_area<<endl;
                hit_count = 1;
                Point2f p[4];
                r.points(p);
                Point2f src_p[4];

                if (r_h>r_w)
                {
                    src_p[0] = p[0];
                    src_p[1] = p[3];
                    src_p[2] = p[2];
                    src_p[3] = p[1];
                }
                else
                {
                    src_p[0] = p[1];
                    src_p[1] = p[0];
                    src_p[2] = p[3];
                    src_p[3] = p[2];
                }
                Mat dst;
                //            for (int i=0;i<1000;i++)
                //            {
                //                double t = (double)getTickCount();
                Mat matrix_per = getPerspectiveTransform(src_p,dst_p);
                warpPerspective(max_color,dst,matrix_per,Size(60,30));
                Mat l_match_result,r_match_result;
                double min_l,max_l,min_r,max_r,last_max;
                Point lmin_loc,lmax_loc,rmin_loc,rmax_loc;
                matchTemplate(dst,l_uhit,l_match_result,TM_CCORR_NORMED);
                matchTemplate(dst,r_uhit,r_match_result,TM_CCORR_NORMED);
                minMaxLoc(l_match_result,&min_l,&max_l,&lmin_loc,&lmax_loc);
                minMaxLoc(r_match_result,&min_r,&max_r,&rmin_loc,&rmax_loc);
                cout<<"uh_value_l:"<<max_l<<endl;
                cout<<"uh_value_r:"<<max_r<<endl;
                last_max = max_l > max_r ? max_l:max_r;
                if (last_max > energy_threshold)
                {
                    found = 1;
                    min_index = i;
                    result = dst.clone();
                    re_p[0] = src_p[0];
                    re_p[1] = src_p[1];
                    re_p[2] = src_p[2];
                    re_p[3] = src_p[3];
                }
            }
            else if(co_area/wh_area > 0.68)
            {

                Point2f p[4];
                r.points(p);
                Point2f src_p[4];
                if (r_h>r_w)
                {
                    src_p[0] = p[0];
                    src_p[1] = p[3];
                    src_p[2] = p[2];
                    src_p[3] = p[1];
                }
                else
                {
                    src_p[0] = p[1];
                    src_p[1] = p[0];
                    src_p[2] = p[3];
                    src_p[3] = p[2];
                }
                Mat dst;
                Mat matrix_per = getPerspectiveTransform(src_p,dst_p);
                warpPerspective(max_color,dst,matrix_per,Size(60,30));
                Mat l_match_result,r_match_result;
                double min_l,max_l,min_r,max_r,last_max;
                Point lmin_loc,lmax_loc,rmin_loc,rmax_loc;
                matchTemplate(dst,l_hit,l_match_result,TM_CCORR_NORMED);
                matchTemplate(dst,r_hit,r_match_result,TM_CCORR_NORMED);
                minMaxLoc(l_match_result,&min_l,&max_l,&lmin_loc,&lmax_loc);
                minMaxLoc(r_match_result,&min_r,&max_r,&rmin_loc,&rmax_loc);
                cout<<"h_value_l:"<<max_l<<endl;
                cout<<"h_value_r:"<<max_r<<endl;
                last_max = max_l > max_r ? max_l:max_r;
                if (last_max > energy_threshold)
                {
                    recent_hited++;
                }
            }

        }
    }
    //    cout<<hited_count<<endl;
    imshow("result",result);
    //    cout<<recent_hited<<endl;

    if ((change_aim == 3)&&(recent_hited!=0))
    {
        change_aim = 3;
    }
    else
    {
        if (recent_hited == hited_count)
        {
            change_aim = 0;//0是不需要更换目标
        }
        else if (recent_hited-hited_count == 1)
        {
            if (recent_hited == 5)
            {
                change_aim = 3;
            }
            else
            {
                change_aim = 1;//1是需要更换目标且是打中而更换目标
            }
        }
        else
        {
            change_aim = 2;//2是需要更换目标且是超时间没打中需要更换目标
        }
    }
    hited_count = recent_hited;
    //    cout<<recent_hited<<endl;


    if (hit_count == 0)
    {
        energyInf.c_rect = RotatedRect();
        energyInf.re_aim = Point2f();
        return energyInf;
    }
#ifdef SHOW_FU
    RotatedRect rect = minAreaRect(cnt[min_index]);
    Point2f pp[4];
    rect.points(pp);
    for (int i=0;i<4;i++)
    {
        line(src,pp[i],pp[(i+1)%4],Scalar(255,0,0),2);
    }
#endif

    std::vector<std::vector<Point2f>> re_cnt;
    std::vector<Vec4i> hei;

    findContours(result,re_cnt,hei,RETR_CCOMP,CHAIN_APPROX_SIMPLE);

    Point2f aim;
    RotatedRect aim_rect;
    for (int i=0;i<re_cnt.size();i++)
    {
        //                    std::cout<<contourArea(re_cnt[i])<<std::endl;
        bool limit1 = hei[i][2] == -1;
        bool limit2 = contourArea(re_cnt[i]) > 130;
        bool limit3 = hei[i][3] !=-1;
        if (limit1 && limit2 && limit3)
        {
            aim_rect = minAreaRect(re_cnt[i]);
            aim.x = aim_rect.center.x;
            aim.y = aim_rect.center.y;
            //            circle(result,aim_rect.center,3,Scalar(255),-1);
            break;
        }
    }
    if (aim_rect.size.empty())
    {
        energyInf.c_rect = RotatedRect();
        energyInf.re_aim = Point2f();
        return energyInf;
    }

    std::vector<Point2f> warp_aim_vec;
    std::vector<Point2f> unwarp_aim_vec;
    Point2f re_aim;
    warp_aim_vec.push_back(aim);
    Mat unmatrix_warp = getPerspectiveTransform(dst_p,re_p);
    perspectiveTransform(warp_aim_vec,unwarp_aim_vec,unmatrix_warp);
    re_aim = unwarp_aim_vec[0];
#ifdef AIM_SHOW
    circle(src,re_aim,5,Scalar(255,0,0),-1);
#endif
    RotatedRect unwarp_rect = minAreaRect(cnt[min_index]);
    Point2f unwarp_p[4];
    unwarp_rect.points(unwarp_p);
    Point2f need_p;
    if (unwarp_rect.size.height > unwarp_rect.size.width)
    {
        if (aim.x<30)
        {
            need_p.x = (unwarp_p[1].x + unwarp_p[2].x)/2;
            need_p.y = (unwarp_p[1].y + unwarp_p[2].y)/2;
        }
        else
        {
            need_p.x = (unwarp_p[0].x + unwarp_p[3].x)/2;
            need_p.y = (unwarp_p[0].y + unwarp_p[3].y)/2;
        }
    }
    else
    {
        if (aim.x<30)
        {
            need_p.x = (unwarp_p[2].x + unwarp_p[3].x)/2;
            need_p.y = (unwarp_p[2].y + unwarp_p[3].y)/2;
        }
        else
        {
            need_p.x = (unwarp_p[0].x + unwarp_p[1].x)/2;
            need_p.y = (unwarp_p[0].y + unwarp_p[1].y)/2;
        }
    }
    Point2f R_center;
    R_center.x = re_aim.x + (need_p.x - re_aim.x)*2/1.48;
    R_center.y = re_aim.y + (need_p.y - re_aim.y)*2/1.48;

    Rect c_roi;
    c_roi.x = R_center.x - 22;//原来是22
    c_roi.y = R_center.y - 22;
    c_roi.width = 44;
    c_roi.height = 44;
    if ((c_roi.x<0)||(c_roi.x>src.cols)||(c_roi.y<0)||(c_roi.y>src.rows))
    {
        energyInf.c_rect = RotatedRect();
        energyInf.re_aim = Point2f();
        return energyInf;
    }
#ifdef R_ROI_SHOW
    circle(src,R_center,5,Scalar(255,255,0),-1);
    rectangle(src,c_roi,Scalar(255,0,255),2);
#endif
    std::vector<std::vector<Point2f>> c_cnt;
    findContours(center_mat(c_roi),c_cnt,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
    if(c_cnt.size()==0)
    {
        energyInf.c_rect = RotatedRect();
        energyInf.re_aim = Point2f();
        return energyInf;
    }
    int max_c = 0;
    for (int i=0;i<c_cnt.size();i++)
    {
        if (contourArea(c_cnt[i]) > contourArea(c_cnt[max_c]))
        {
            max_c = i;
        }
    }
    RotatedRect c_rect = minAreaRect(c_cnt[max_c]);
    //    std::cout<<c_rect.size.width*c_rect.size.height<<std::endl;
    if (c_rect.size.width*c_rect.size.height < 170)
    {
        energyInf.c_rect = RotatedRect();
        energyInf.re_aim = Point2f();
        return energyInf;
    }
    Rect pnp_rect = boundingRect(c_cnt[max_c]);
    pnp_rect.x = pnp_rect.x + c_roi.x;
    pnp_rect.y = pnp_rect.y + c_roi.y;
    Eigen::Vector3d real_c = pnp_get(pnp_rect);
    //    std::cout<<real_c(1,0)<<'\t'<<real_c(0,0)<<std::endl;
    get_ap(real_c,re_aim,c_rect);
    double recent_depth = real_c(2,0);
    distances.push_back(recent_depth);
    if (distances.size() == 11)
    {
        depth = depth*0.3 + 0.7*depth_filter(distances);
    }
    else
    {
        depth = depth*0.3 + 0.7*recent_depth;
    }
    //    std::cout<<depth<<std::endl;
#ifdef R_SHOW
    c_rect.center.x = c_rect.center.x + c_roi.x;
    c_rect.center.y = c_rect.center.y + c_roi.y;
    circle(src,c_rect.center,5,Scalar(255,0,0),-1);
#endif
    energyInf.re_aim = re_aim;
    energyInf.c_rect = c_rect;
#ifdef R_RECT_SHOW
    Point2f cp[4];
    c_rect.points(cp);
    for (int i=0;i<4;i++)
    {
        line(src,cp[i],cp[(i+1)%4],Scalar(255,0,0),2);
    }
#endif
    imshow("src",src);
    std::cout<<"one frame"<<std::endl;
    return energyInf;
}

void energy::show_all_dst()
{
    for (int i=0;i<warp_vec.size();i++)
    {
        imshow("warp",warp_vec[i]);
    }
}


Eigen::Vector3d energy::pnp_get(Rect &c_rect)
{
    double x = c_rect.x;
    double y = c_rect.y;
    double w = c_rect.width;
    double h = c_rect.height;
    Point2f lu,ld,ru,rd;
    std::vector<cv::Point3d> ps = {
            {-0.106 / 2 , -0.106 / 2, 0.},
            {0.106 / 2 , -0.106 / 2, 0.},
            {0.106 / 2 , 0.106 / 2, 0.},
            {-0.106 / 2 , 0.106 / 2, 0.}
    };
    lu = Point2f(x,y);
    ld = Point2f(x,y+h);
    ru = Point2f(x+w,y);
    rd = Point2f(x+w,y+h);

    std::vector<cv::Point2f> pu;
    pu.push_back(lu);
    pu.push_back(ru);
    pu.push_back(rd);
    pu.push_back(ld);

    Mat rvec;
    Mat tvec;
    Eigen::Vector3d tv;


    F_MAT=(cv::Mat_<double>(3, 3) << 1554.52600, 0.000000000000, 630.01725, 0.000000000000, 1554.47451, 519.78242, 0.000000000000, 0.000000000000, 1.000000000000);
    C_MAT=(cv::Mat_<double>(1, 5) << -0.08424, 0.16737, -0.00006, 0.00014, 0.00000);

    cv::solvePnP(ps, pu, F_MAT, C_MAT, rvec, tvec);


    cv2eigen(tvec, tv);//这个转为了啥

    //            std::cout<<tv(2,0)<<std::endl;
    return tv;
}

void energy::get_ap(Eigen::Vector3d &real_c,Point2f &Aim,RotatedRect &c_rect)
{
    Point2f R_center = c_rect.center;
    int r_x = R_center.x;
    int r_y = R_center.y;
    int a_x = Aim.x;
    int a_y = Aim.y;
    double ar_dis = sqrt((a_x-r_x)*(a_x-r_x)+(a_y-r_y)*(a_y-r_y));
    double ar_xdis = a_x-r_x;
    double ar_ydis = a_y-r_y;
    double cos_ar = ar_xdis/ar_dis;
    double sin_ar = ar_ydis/ar_dis;
    real_xy[0] = real_c(0,0)+0.7*cos_ar;
    real_xy[1] = real_c(1,0)+0.7*sin_ar;
    //    cout<<real_xy[0]<<"\t"<<real_xy[1]<<endl;

}

double energy::depth_filter(deque<double> &dis)
{
    deque<double> filter;
    swap(dis,filter);
    sort(filter.begin(),filter.end());
    dis.pop_front();
    return filter[5];
}

void energy::make_c_safe(cv::Rect &line)
{
    if (line.x<0)
    {
        line.x=0;
    }
    else if (line.x>src.cols)
    {
        line.x=src.cols;
    }
    if (line.y<0)
    {
        line.y=0;
    }
    else if (line.y>src.rows)
    {
        line.y=src.rows;
    }

    if (line.x-line.width<0)
    {
        line.width = 2*line.x;
    }
    if (line.y-line.height<0)
    {
        line.height = 2*line.y;
    }

    if (line.x+line.width>src.cols)
    {
        line.width=2*(src.cols-line.x);
    }
    if (line.y+line.height>src.rows)
    {
        line.height=2*(src.rows-line.y);
    }
}

