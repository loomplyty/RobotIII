/* 步态功能：通过三点触碰找到孔中心，进行插孔
 * by liujimu, 2016-4-26
 */

/*将以下注释代码添加到xml文件*/
/*
            <ph default="ph_param">
                <ph_param type="group">
                    <totalCount abbreviation="t" type="int" default="2000"/>
                    <holeDepth abbreviation="d" type="double" default="0.1"/>
                    <contactForce abbreviation="f" type="double" default="20"/>
                </ph_param>
            </ph>
            <phc/>
*/

#ifndef PEG_IN_HOLE_H
#define PEG_IN_HOLE_H

#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <cstring>
#include <map>
#include <string>
#include <stdlib.h>
#include <atomic>

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>

#ifndef PI
#define PI 3.141592653589793
#endif

enum class PegInHoleProcess
{
    PREPARE = 0,
    ALIGN = 1,
    INSERT = 2,
    RETURN = 3,
    STOP = 4,
};

/*gait parameters*/
struct phParam final:public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 2000 }; //插入过程所需时间
    double holeDepth{ 0.1 }; //插孔深度
    double contactForce{ 20 }; //接触力判断阈值
};

struct mbParam final:public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 2000 };
    double targetPeb[6]{ 0 };
    //bool isAbsolute{ false }; //用于判断移动命令是绝对坐标还是相对坐标
    aris::dynamic::FloatMarker *beginMak{ nullptr };
};

/*parse function*/
auto pegInHoleParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;

/*operation function*/
auto pegInHoleGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;
auto moveBodyGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // PEG_IN_HOLE_H
