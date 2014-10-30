//----------------------------------------------------------------------------------
//
// Copyright (c) 2013, NVIDIA CORPORATION. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//
//----------------------------------------------------------------------------------


#ifndef SHADERPROGRAM_H_
#define SHADERPROGRAM_H_

#include <map>
#include <string>
#include <GL/glew.h>
#include "planecalib/log.h"
#include "planecalib/eutils.h"

namespace planecalib
{

class ShaderProgram
{
public:
    ShaderProgram();
    ~ShaderProgram();

    static bool CreateFragmentProgramFromStrings(GLuint programId, GLuint vertexShaderId, GLuint fragmentShaderId,
                                                 const char *vertexShader, const char *fragmentShader);
    static bool CreateFragmentProgram(GLuint &destProgramId, const char *vertexShader, const char *fragmentShader);
    static bool LoadFragmentProgram(GLuint &programId, const char *vertexShaderFileName,
                                        const char *fragmentShaderFileName);

    bool load(const char *vertexShaderFilename, const char *fragmentShaderFilename,
    		const char *uniformNames[], int *uniformIds[], int uniformCount,
    		const char *attribNames[], int *attribIds[], int attribCount);
    void free();

    bool isLoaded() const { return mLoaded; }
    unsigned int getId() const {return mId; }

protected:
    bool mLoaded;
    unsigned int mId;
};

}

#endif /* SHADERPROGRAM_H_ */
