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

#ifndef PMM_DIRECTIONALDATAANDRGB_H_
#define PMM_DIRECTIONALDATAANDRGB_H_

#include "pmm.h"

#include <string>
#include <sstream>

PMM_NAMESPACE_BEGIN

struct DirectionalDataAndRGB
{
    float weight;
    Point3 position;
    Vector3 direction;
    Vector3 rgb;
    float distance;
    uint32_t flags;

    const std::string toString()const{
        std::ostringstream oss;
        oss << "DirectionalDataAndRGB[" << endl
            << "  weight = " << weight << endl
            << "  position = [" << position[0] << "\t" << position[1] << "\t" << position[2] << "]" << endl
            << "  direction = [" << direction[0] << "\t" << direction[1] << "\t" << direction[2] << "]" << endl
            << "  distance = " << distance << endl
            << "  flags = " << flags << '\n'
            << "]";
        return oss.str();
    }


    bool operator<(const DirectionalDataAndRGB& comp) const
    {
        return    weight < comp.weight ||
                ( weight        == comp.weight        && ( position[0] < comp.position[0]   ||
                ( position[0]   == comp.position[0]   && ( position[1] < comp.position[1]   ||
                ( position[1]   == comp.position[1]   && ( position[2] < comp.position[2]   ||
                ( position[2]   == comp.position[2]   && ( direction[0]< comp.direction[0]  ||
                ( direction[0]  == comp.direction[0]  && ( direction[1]< comp.direction[1]  ||
                ( direction[1]  == comp.direction[1]  && ( direction[2]< comp.direction[2]))))))))))));
    }
};

PMM_NAMESPACE_END

#endif
