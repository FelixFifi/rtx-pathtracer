///////////////////////////////////////////////////////////////////////////
// 
// Copyright (c) 2020, Sebastian Herholz, Lukas Ruppert
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the author nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
///////////////////////////////////////////////////////////////////////////

#ifndef PMM_DIRECTIONALDATA_H_
#define PMM_DIRECTIONALDATA_H_

#include "pmm.h"

#include <iostream>
#include <fstream>
#include <vector>

PMM_NAMESPACE_BEGIN


/// The data structure to represent the samples (e.g., photons) 
/// used for fitting the directional mixture models.
struct DirectionalData
{
    // the positin of the sample in 3D (e.g, photon position)
    Point3 position;
    // direction of the sample (e.g., direction pointing to the origin of the photon)
    Vector3 direction;
    // the weight or value associated to this direction (e.g., photon power)
    float weight;
    // the PDF for sampling/generating this directinal sample
    float pdf;
    // the distance associated to the directional sample (e.g., distance to the photon's origin)
    float distance;
    uint32_t flags;

    DirectionalData() = default;
    DirectionalData(Point3 position, Vector3 direction, float weight, float pdf, float distance, uint32_t flags = 0)
        : position(position), direction(direction), weight(weight), pdf(pdf), distance(distance), flags(flags)
    {
    }

    std::string toString()const{
        std::ostringstream oss;
        oss << "DirectionalData[\n"
            << "  position = [" << position[0] << ", " << position[1] << ", " << position[2] << "]\n"
            << "  direction = [" << direction[0] << ", " << direction[1] << ", " << direction[2] << "]\n"
            << "  weight = " << weight << '\n'
            << "  pdf = " << pdf << '\n'
            << "  distance = " << distance << '\n'
            << "  flags = " << flags << '\n'
            << "]";
        return oss.str();
    }

    bool operator==(const DirectionalData& comp) const
    {
        return (position  == comp.position  &&
                direction == comp.direction &&
                weight    == comp.weight    &&
                pdf       == comp.pdf       &&
                distance  == comp.distance  &&
                flags     == comp.flags);
    }

    bool operator<(const DirectionalData& comp) const
    {
        return                                             weight       < comp.weight ||
                ( weight        == comp.weight        && ( pdf          < comp.pdf           ||
                ( pdf           == comp.pdf           && ( position[0]  < comp.position[0]   ||
                ( position[0]   == comp.position[0]   && ( position[1]  < comp.position[1]   ||
                ( position[1]   == comp.position[1]   && ( position[2]  < comp.position[2]   ||
                ( position[2]   == comp.position[2]   && ( direction[0] < comp.direction[0]  ||
                ( direction[0]  == comp.direction[0]  && ( direction[1] < comp.direction[1]  ||
                ( direction[1]  == comp.direction[1]  && ( direction[2] < comp.direction[2]  ||
                ( direction[2]  == comp.direction[2]  && ( distance     < comp.distance      ||
                ( distance      == comp.distance      && ( flags        < comp.flags
                ))))))))))))))))));
    }
};

template<typename TData>
void storeDirectionalData(const std::string &fileName,
                          typename std::vector<TData>::const_iterator begin,
                          typename std::vector<TData>::const_iterator end){
    std::ofstream file;
    file.open(fileName, std::ios::binary);

    ptrdiff_t nData = std::distance(begin, end);
    file.write(reinterpret_cast<const char*>(&nData), sizeof(nData));
    file.write(reinterpret_cast<const char*>(&*begin), sizeof(TData)*nData);

    file.close();
}

template<typename TData>
void storeDirectionalData(const std::string &fileName, const std::vector<TData> &ddata){
    storeDirectionalData<TData>(fileName, ddata.cbegin(), ddata.cend());
}

template<typename TData>
std::vector<TData> loadDirectionalData(const std::string &fileName){
    std::vector<TData> ddata;

    std::ifstream file;
    file.open(fileName, std::ios::binary);

    ptrdiff_t nData;
    file.read(reinterpret_cast<char*>(&nData), sizeof(nData));
    std::cout << "lightpmm::loadDirectionalData: nData: " << nData << std::endl;

    ddata.resize(nData);
    file.read(reinterpret_cast<char*>(ddata.data()), sizeof(TData)*nData);

    file.close();
    return ddata;
}


PMM_NAMESPACE_END

#endif
