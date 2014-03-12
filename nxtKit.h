#ifndef NXT_KIT_H
#define NXT_KIT_H

#include "baxterLimb.h"


struct Container{
    cv::RotatedRect rect;
    std::vector<std::string> objects;
};

class NxtKit{
public:
    NxtKit(cv::RotatedRect, double);
    cv::RotatedRect get_coordinates(std::string);
    double height();
    double angle();
    gv::Point origin();
    
    void show_parts();
    void show_containers();
    
    bool contains(std::string); // checks if the kit contains the named part

private:
    NxtKit();
    
    void _setup_containers();
    cv::RotatedRect _to_rect(cv::Point2f, cv::Size2f);
    cv::RotatedRect _to_global(Container);
    
    cv::RotatedRect _box;
    gv::Point _origin;                  //  x, y position shared with _box
    std::vector<Container> _containers; //  positions relative to origin of box
    double _angle;                      //  yaw of box

};

#endif

NxtKit::NxtKit(cv::RotatedRect rect, double height)
{
    _box = rect;
    cv::Point2f points[4];
    rect.points(points);
    _origin.x = points[0].x;
    _origin.y = points[0].y;
    _origin.z = height;
    _origin.print("NXT Kit Origin");
    _angle = rect.angle*PI/180;
    _setup_containers();
}

void NxtKit::_setup_containers()
{
    Container _container;
    std::vector<std::string> objs;
    _container.rect = _to_rect(cv::Point2f(.015,.015), cv::Size2f(.085,.075));
    objs.push_back("lift_arm");                 //4563045
    objs.push_back("double_pin");               //4119589
    objs.push_back("pulley_wheel");             //4494222
    objs.push_back("pulley_tire");              //281526
    objs.push_back("double_pin_connector");     //4225033
    objs.push_back("oval_brace");               //4239896
    _container.objects = objs;
    _containers.push_back(_container);
    objs.clear();
    // OBSCURE JOINERS/BRACKETS
    _container.rect = _to_rect(cv::Point2f(.015,.095), cv::Size2f(.08,.08));
    objs.push_back("4211775");
    objs.push_back("4210857");
    objs.push_back("4512363");  // I think this part doesn't exist
    objs.push_back("4121667");
    objs.push_back("4210655");
    objs.push_back("4107783");
    objs.push_back("4296059");  // I think this part doesn't exist
    objs.push_back("4113805");
    _container.objects = objs;
    _containers.push_back(_container);
    objs.clear();
    // GEARS
    _container.rect = _to_rect(cv::Point2f(.015,.18), cv::Size2f(.08,.074));
    objs.push_back("8_tooth_gear");     //4514559
    objs.push_back("16_tooth_gear");    //4562487
    objs.push_back("crown_gear");       //4211434
    objs.push_back("40_tooth_gear");    //4285634
    objs.push_back("24_tooth_gear");    //4514558
    objs.push_back("20_tooth_gear");    //4514555
    objs.push_back("12_tooth_gear");    //4177431
    objs.push_back("worm_gear");        //4211510
    objs.push_back("knob_wheel");       //4248204
    objs.push_back("36_tooth_gear");    //4255563
    _container.objects = objs;
    _containers.push_back(_container);
    objs.clear();
    // AXLES
    _container.rect = _to_rect(cv::Point2f(.106,.015), cv::Size2f(.041,.121));
    objs.push_back("2_axle");           //4142865
    objs.push_back("4_axle");           //370526
    objs.push_back("6_axle");           //370626
    objs.push_back("8_axle");           //370726
    objs.push_back("10_axle");          //373726
    objs.push_back("12_axle");          //370826
    objs.push_back("3_axle");           //4211815
    objs.push_back("5_axle");           //4211639
    objs.push_back("divided_axle");     //4508553
    _container.objects = objs;
    _containers.push_back(_container);
    objs.clear();
    // AXLEPEGS
    _container.rect = _to_rect(cv::Point2f(.102,.142), cv::Size2f(.035,.054));
    objs.push_back("axle_peg");         //4186017
    objs.push_back("axle_peg_friction");//4206482
    objs.push_back("extended_axle_peg");//4140801
    _container.objects = objs;
    _containers.push_back(_container);
    objs.clear();
    // PEGS
    _container.rect = _to_rect(cv::Point2f(.102,.201), cv::Size2f(.045,.054));
    objs.push_back("peg");      //4121715
    _container.objects = objs;
    _containers.push_back(_container);
    objs.clear();
    // MINIFIG, ETC
    _container.rect = _to_rect(cv::Point2f(.154,.015), cv::Size2f(.039,.121));
    objs.push_back("minifig");
    objs.push_back("small_wheel");
    objs.push_back("small tire");
    objs.push_back("light");
    objs.push_back("2_brick_red");
    objs.push_back("2_brick_green");
    objs.push_back("2_brick_yellow");
    objs.push_back("pulley_band_yellow");
    objs.push_back("pulley_band_red");
    _container.objects = objs;
    _containers.push_back(_container);
    objs.clear();
    // EXTENDED PEGS
    _container.rect = _to_rect(cv::Point2f(.145,.141), cv::Size2f(.044,.058));
    objs.push_back("extended_peg");     //655826
    _container.objects = objs;
    _containers.push_back(_container);
    objs.clear();
    // BUSHINGS
    _container.rect = _to_rect(cv::Point2f(.155,.201), cv::Size2f(.034,.054));
    objs.push_back("bushing");          //4211622
    objs.push_back("half_bushing");     //4239601
    _container.objects = objs;
    _containers.push_back(_container);
    objs.clear();
    // STRAIGHT BEAMS
    _container.rect = _to_rect(cv::Point2f(.2,.015), cv::Size2f(.074,.133));
    objs.push_back("3_beam");           //4210751
    objs.push_back("5_beam");           //4210686
    objs.push_back("7_beam");           //4495931
    objs.push_back("9_beam");           //4210757
    objs.push_back("11_beam");          //4210755
    objs.push_back("13_beam");          //4522947
    objs.push_back("15_beam");          //4542576
    _container.objects = objs;
    _containers.push_back(_container);
    objs.clear();
    // BENT BEAMS
    _container.rect = _to_rect(cv::Point2f(.195,.154), cv::Size2f(.09,.102));
    objs.push_back("3_5_beam");         //4210753
    objs.push_back("2_4_beam");         //4210667
    objs.push_back("4_6_beam");         //4210638
    objs.push_back("3_4_7_beam");       //4210668
    _container.objects = objs;
    _containers.push_back(_container);
    objs.clear();
    // BRICKS/PLATES
    _container.rect = _to_rect(cv::Point2f(.283,.015), cv::Size2f(.085,.102));
    objs.push_back("2_1_plate");        //4211398
    objs.push_back("2_1_brick");        //4211388
    objs.push_back("2_2_brick");        //4211387
    objs.push_back("4_1_plate");        //4211445
    objs.push_back("4_2_plate");        //4211444
    objs.push_back("6_2_plate");        //4211542
    objs.push_back("8_2_plate");        //4211449
    objs.push_back("2_1_smooth_plate"); //4211052
    _container.objects = objs;
    _containers.push_back(_container);
    objs.clear();
    // PEG BRICKS
    _container.rect = _to_rect(cv::Point2f(.292,.122), cv::Size2f(.076,.134));
    objs.push_back("2_1_axle_brick");   //4210935
    objs.push_back("2_1_peg_brick");    //4211440
    objs.push_back("4_1_peg_brick");    //4211441
    objs.push_back("6_1_peg_brick");    //4211466
    objs.push_back("8_1_peg_brick");    //4211442
    objs.push_back("16_1_peg_brick");   //4211443
    _container.objects = objs;
    _containers.push_back(_container);
    objs.clear();
}

void NxtKit::show_parts() 
{
    std::cout<<"\n";
    for(int i = 0; i < _containers.size(); i++)
    {
        for(int j = 0; j < _containers[i].objects.size(); j++)
        {
            std::cout<<_containers[i].objects[j]<<"\n";
        }
    }
}

void NxtKit::show_containers()
{
    std::cout<<"\n";
    for(int i = 0; i < _containers.size(); i++)
        std::cout << _containers[i].rect.center << "\n";
}

cv::RotatedRect NxtKit::_to_rect(cv::Point2f origin, cv::Size2f size)
{
    cv::RotatedRect rect;
    rect.angle = _box.angle;
    rect.center.x = origin.x + size.width/2;
    rect.center.y = origin.y + size.height/2;
    rect.size = size;
    return rect;
}

cv::RotatedRect NxtKit::get_coordinates(std::string part_name)
{
    for(int i = 0; i < _containers.size(); i++)
    {
        for(int j = 0; j < _containers[i].objects.size(); j++)
        {
            if(part_name == _containers[i].objects[j])
                return _to_global( _containers[i] );
        }
    }
    return cv::RotatedRect(cv::Point2f(-1,-1), cv::Size2f(-1,-1), 0);
}

cv::RotatedRect NxtKit::_to_global(Container container) 
{
    // returns coordinates of container relative to Baxter
    //std::cout<<" rect center "<<container.rect.center<<"\n";
    double xTemp = container.rect.center.x;
    double yTemp = container.rect.center.y;
    double x = cos(-_angle)*xTemp - sin(-_angle)*yTemp;
    double y = -sin(-_angle)*xTemp - cos(-_angle)*yTemp;
    //std::cout<<" relative -- "<<x<<" "<<y<<"\n";
    x += _origin.x;
    y += _origin.y;
    container.rect.center = cv::Point2f(x,y);
    
    return container.rect;
}

bool NxtKit::contains(std::string part_name) 
{
    for(int i = 0; i < _containers.size(); i++)
    {
        for(int j = 0; j < _containers[i].objects.size(); j++)
        {
            if(part_name == _containers[i].objects[j])
                return true;
        }
    }
    return false;
}

double NxtKit::height()
{
    return _origin.z;
}

gv::Point NxtKit::origin()
{
    return _origin;
}

double NxtKit::angle()
{
    return _angle;
}
