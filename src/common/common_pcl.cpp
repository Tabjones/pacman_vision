// Software License Agreement (BSD License)
//
//   PaCMan Vision (PaCV) - https://github.com/Tabjones/pacman_vision
//   Copyright (c) 2015-2016, Federico Spinelli (fspinelli@gmail.com)
//   All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder(s) nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include <common/common_pcl.h>
#include <pcl/console/print.h>

namespace pacv
{
void
crop_a_box(const PTC::ConstPtr source, PTC::Ptr& dest, const Box lim, const bool remove_inside,
        const Eigen::Matrix4f &trans, const bool keep_organized)
{
    if(!source){
        pcl::console::print_warn("[%s]\tUninitialized source cloud pointer recived, not doing anything...",__func__);
        return;
    }
    if(source->empty()){
        pcl::console::print_warn("[%s]\tEmpty source cloud recived, not doing anything...",__func__);
        return;
    }
    if(!dest)
        dest=boost::make_shared<PTC>();
    pcl::CropBox<PT> cb;
    cb.setKeepOrganized(keep_organized);
    cb.setInputCloud(source);
    Eigen::Vector4f min,max;
    min << lim.x1, lim.y1, lim.z1, 1;
    max << lim.x2, lim.y2, lim.z2, 1;
    cb.setMin(min);
    cb.setMax(max);
    //Note this transform is applied to the box, not the cloud
    if(!trans.isIdentity()){
        Eigen::Affine3f t (trans);
        cb.setTransform(t);
    }
    cb.setNegative(remove_inside);
    cb.filter (*dest);
}
void
crop_a_box(const PXC::ConstPtr source, PXC::Ptr& dest, const Box lim, const bool remove_inside,
        const Eigen::Matrix4f &trans, const bool keep_organized)
{
    if(!source){
        pcl::console::print_warn("[%s]\tUninitialized source cloud pointer recived, not doing anything...",__func__);
        return;
    }
    if(source->empty()){
        pcl::console::print_warn("[%s]\tEmpty source cloud recived, not doing anything...",__func__);
        return;
    }
    if(!dest)
        dest=boost::make_shared<PXC>();
    pcl::CropBox<PX> cb;
    cb.setKeepOrganized(keep_organized);
    cb.setInputCloud(source);
    Eigen::Vector4f min,max;
    min << lim.x1, lim.y1, lim.z1, 1;
    max << lim.x2, lim.y2, lim.z2, 1;
    cb.setMin(min);
    cb.setMax(max);
    //Note this transform is applied to the box, not the cloud
    if(!trans.isIdentity()){
        Eigen::Matrix3f Rot = trans.block<3,3>(0,0); //3x3 block starting at 0,0
        Eigen::Vector3f angles = Rot.eulerAngles(0,1,2);
        Eigen::Vector3f translation( trans(0,3), trans(1,3), trans(2,3));
        cb.setTranslation(translation);
        cb.setRotation(angles);
    }
    cb.setNegative(remove_inside);
    cb.filter (*dest);
}

void castUint8ToDouble(uint8_t b, double &d)
{
    uint64_t b_64 = (uint64_t)b;
    d = static_cast<double>(b_64);
}

void castPCLColorToDouble(const PT &pt, double &r, double &g, double &b)
{
    castUint8ToDouble(pt.r, r);
    castUint8ToDouble(pt.g, g);
    castUint8ToDouble(pt.b, b);
}

inline double labF(const double t)
{
    return ( (t >= 8.85645167903563082e-3) ? std::pow(t,0.333333333333333) : (841.0/108.0)*t + (4.0/29.0) );
}

inline double invGammaCorrection(const double t)
{
    return (t <= 0.0404482362771076 ? t/12.92 : std::pow( (t+0.055)/1.055, 2.4 ));
}

void rgb2Xyz(double r, double g, double b, double &x, double &y, double &z)
{
    //Perform the inverse gamma correction on all channels
    r = invGammaCorrection(r);
    g = invGammaCorrection(g);
    b = invGammaCorrection(b);
    //conversion
    x = (double)(0.4123955889674142161*r + 0.3575834307637148171*g + 0.1804926473817015735*b);
    y = (double)(0.2125862307855955516*r + 0.7151703037034108499*g + 0.07220049864333622685*b);
    z = (double)(0.01929721549174694484*r + 0.1191838645808485318*g + 0.9504971251315797660*b);
}

void xyz2Lab(double x, double y, double z, double &L, double &a,  double &b)
{
    x /= WHITEPOINT_X;
    y /= WHITEPOINT_Y;
    z /= WHITEPOINT_Z;
    x = labF(x);
    y = labF(y);
    z = labF(z);
    L = 116*y - 16;
    a = 500*(x - y);
    b = 200*(y - z);
}

void convertPCLColorToCIELAB(const PT &pt, double &L, double &a, double &b)
{
    double r,g,bp;
    //get colors to double in 0-255 range
    castPCLColorToDouble(pt, r,g,bp);
    //normalize them to 0-1
    r/=255;
    g/=255;
    bp/=255;
    convertRGBToCIELAB(r,g,bp, L,a,b);
}
void convertRGBToCIELAB(double rr, double gg, double bb, double &L, double &a, double &b)
{
    double x,y,z;
    //convert to XYZ colorspace
    rgb2Xyz(rr,gg,bb, x,y,z);
    //finally get CIEL*a*b* colorspace
    xyz2Lab(x,y,z, L,a,b);
}

double deltaE(const double L1, const double a1, const double b1,
              const double L2, const double a2, const double b2,
              const double Kl, const double Kc, const double Kh,
              const bool verbose)
{
    //compute Cab(s)
    double C1ab = std::sqrt( std::pow(a1,2) + std::pow(b1,2) );
    double C2ab = std::sqrt( std::pow(a2,2) + std::pow(b2,2) );
    double mCab = (C1ab + C2ab)*0.5;
    //compute G
    double G= 0.5*(1 - std::sqrt( std::pow(mCab,7)/(std::pow(mCab,7) + std::pow(25,7)) ) );
    //compute a primes
    double a1prime = (1 + G)*a1;
    double a2prime = (1 + G)*a2;
    //compute C primes
    double C1prime = std::sqrt( std::pow(a1prime,2) + std::pow(b1,2) );
    double C2prime = std::sqrt( std::pow(a2prime,2) + std::pow(b2,2) );
    //compute h primes and convert them in 0-360 deg ranges
    double h1prime = (b1 == a1prime && b1 == 0) ? 0 : std::atan2(b1, a1prime);
    h1prime = h1prime <0 ? h1prime + 2*M_PI : h1prime;
    h1prime *= (180/M_PI);
    double h2prime = (b1 == a2prime && b2 == 0) ? 0 : std::atan2(b2, a2prime);
    h2prime = h2prime <0 ? h2prime + 2*M_PI : h2prime;
    h2prime *= (180/M_PI);
    //compute delta L prime (difference must be signed)
    double dLprime = L2 - L1;
    //compute delta C prime (difference must be signed)
    double dCprime = C2prime - C1prime;
    //compute delta H prime
    double dhprime(0.0);
    if ( (C1prime*C2prime) ==0.0)
        dhprime = 0.0;
    else if ( (C1prime*C2prime) !=0.0 && std::abs(h2prime - h1prime) <= 180.0)
        dhprime = h2prime - h1prime;
    else if ( (C1prime*C2prime) !=0.0 && (h2prime - h1prime) > 180.0)
        dhprime = (h2prime - h1prime) - 360.0;
    else if ( (C1prime*C2prime) !=0.0 && (h2prime - h1prime) < -180.0)
        dhprime = (h2prime -h1prime) + 360.0;
    else
        //unexpected case
        std::cout<<"[deltaE] Unhandled error in computing delta h prime";
    double dHprime = 2 * std::sqrt(C1prime*C2prime) * std::sin((dhprime*M_PI)/360);
    //compute median L prime
    double mLprime = (L1 + L2)*0.5;
    //compute median C prime
    double mCprime = (C1prime + C2prime)*0.5;
    //compute median h prime
    double mhprime(0.0);
    if (std::abs(h1prime - h2prime) <= 180.0 && (C1prime*C2prime) !=0)
        mhprime = (h1prime+h2prime)*0.5;
    else if (std::abs(h1prime - h2prime) > 180.0 && (h1prime+h2prime) < 360.0 && (C1prime*C2prime) !=0)
        mhprime = (h1prime + h2prime + 360.0)*0.5;
    else if (std::abs(h1prime - h2prime) > 180.0 && (h1prime+h2prime) >= 360.0 && (C1prime*C2prime) !=0)
        mhprime = (h1prime + h2prime - 360.0)*0.5;
    else if ((C1prime*C2prime) == 0.0)
        mhprime = h1prime + h2prime;
    else
        //unexpected case
        std::cout<<"[deltaE] Unhandled error in computing median h prime";
    //compute the T factor
    double T = 1 - 0.17*std::cos((mhprime - 30.0)*M_PI/180) +
                0.24 * std::cos((2*mhprime)*M_PI/180) +
                0.32 * std::cos((3*mhprime + 6.0)*M_PI/180) -
                0.2 * std::cos((4*mhprime - 63.0)*M_PI/180);
    // compute DeltaTheta
    double dtheta = 30.0 * std::exp(-1.0*std::pow((mhprime - 275.0)/25,2));
    //compute Rc
    double Rc = 2.0 * std::sqrt( std::pow(mCprime,7)/(std::pow(mCprime,7) + std::pow(25,7)) );
    //compute Sl
    double Sl = 1.0 + (0.015*std::pow(mLprime - 50.0, 2) / std::sqrt(20.0 + std::pow(mLprime - 50.0,2)));
    //compute Sc, Sh
    double Sc = 1.0 + 0.045*mCprime;
    double Sh = 1.0 + 0.015*mCprime*T;
    //compute Rt
    double Rt = -1.0*std::sin(2.0*dtheta*M_PI/180)*Rc;
    //Finally, compute the delta E color difference
    double deltaE = std::sqrt( std::pow(dLprime/(Kl*Sl),2) +
                       std::pow(dCprime/(Kc*Sc),2) +
                       std::pow(dHprime/(Kh*Sh),2) +
                       Rt * (dCprime/(Kc*Sc)) * (dHprime/(Kh*Sh)) );
    if (verbose){
        std::cout<<"==================================================\n";
        std::cout<<"L1 "<<L1<<" a1 "<<a1<<" b1 "<<b1<<std::endl;
        std::cout<<"L2 "<<L2<<" a2 "<<a2<<" b2 "<<b2<<std::endl;
        std::cout<<"a1' "<<a1prime<<" C1' "<<C1prime<<" h1' "<<h1prime<<std::endl;
        std::cout<<"a2' "<<a2prime<<" C2' "<<C2prime<<" h2' "<<h2prime<<std::endl;
        std::cout<<"_h' "<<mhprime<<" G' "<<G<<" T "<<T<<" Sl "<<Sl<<" Sc "<<Sc<<" Sh "<<Sh<<" Rt "<<Rt<<std::endl;
        std::cout<<"Delta E "<<deltaE<<std::endl;
        std::cout<<"==================================================\n";
    }
    return deltaE;
}

void testDeltaE()
{
    deltaE(50.0, 2.6772, -79.7751, 50.0, 0.0, -82.7485, 1,1,1, true);
    deltaE(50.0, 3.1571, -77.2803, 50.0, 0.0, -82.7485, 1,1,1, true);
    deltaE(50.0, 2.8361, -74.02, 50.0, 0.0, -82.7485, 1,1,1, true);
    deltaE(50.0, -1.3802, -84.2814, 50.0, 0.0, -82.7485, 1,1,1, true);
    deltaE(50.0, 2.5, 0.0, 73.0, 25.0, -18.0, 1,1,1, true);
    deltaE(50.0, 2.5, 0.0, 61.0, -5.0, -3.0, 1,1,1, true);
    deltaE(50.0, 2.5, 0.0, 56.0, -27.0, -3.0, 1,1,1, true);
    deltaE(63.0109, -31.0961, -5.8663, 62.8187, -29.7946, -4.0864, 1,1,1, true);
}

}//namespace
