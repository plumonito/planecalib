/*
 * TextRenderer.cpp
 *
 *  Created on: May 10, 2013
 *      Author: danielh
 */

#include "TextRenderer.h"
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "TextShader.h"

namespace planecalib
{

TextRendererStream::~TextRendererStream()
{
	flush();
}

void TextRendererStream::flush()
{
	if (!mStream.str().empty())
	{
		mRenderer.renderText(mStream);
		mStream.str("");
		mStream.clear();
	}
}

void TextRendererStream::setColor(const Eigen::Vector4f &color) 
{ 
	flush(); 
	mRenderer.setColor(color); 
}

TextRenderer::TextRenderer(){
}

TextRenderer::~TextRenderer()
{
}

bool TextRenderer::init(TextShader *shader)
{
    mShader = shader;
    mFontData.resize(2);
    prepareFontData(mFontData[0], cv::FONT_HERSHEY_SIMPLEX, 0.5, 2, 8);
    prepareFontData(mFontData[1], cv::FONT_HERSHEY_SIMPLEX, 2, 3, 8);
    mActiveFont = &mFontData[0];
    return true;
}

void TextRenderer::prepareFontData(TextFontData &data, int face, double scale, int thickness, int lineType)
{
    //Build the texture used for text rendering
    //Create image with a maximum size
    int basicBaseline;
    cv::Size basicSize = cv::getTextSize("W", face, scale, thickness, &basicBaseline);
    basicSize.height = 3 * basicSize.height;

    cv::Mat1b img = cv::Mat1b::zeros(basicSize.height, basicSize.width * TextFontData::kCharCount);

    //Keep track of each characters size
    int charOrigin[TextFontData::kCharCount];
    cv::Size2i charSize[TextFontData::kCharCount];
    int charBaseline[TextFontData::kCharCount];

    //Add each character
    int maxHeight = 0, maxBaseline = 0;
    int left = 0;
    char str[2] = {0, 0};
    int imgOriginY = 2 * basicSize.height / 3;
    for(int c = 1; c < TextFontData::kCharCount; c++)
    {
        str[0] = c;
        std::string s(str);
        charSize[c] = cv::getTextSize(s, face, scale, thickness, &charBaseline[c]);
        data.mCharAspect[c] = (float)charSize[c].width / charSize[c].height;

        if(charSize[c].height > maxHeight)
        {
            maxHeight = charSize[c].height;
        }
        if(charBaseline[c] > maxBaseline)
        {
            maxBaseline = charBaseline[c];
        }

        charOrigin[c] = left;

        cv::putText(img, s, cv::Point2i(left, imgOriginY), face, scale, cv::Scalar(255), thickness, lineType, false);
        left += charSize[c].width;
    }

    cv::Mat1b imgCrop(img, cv::Rect(0, imgOriginY - maxHeight - 1, left, maxHeight + maxBaseline));
	mRenderCharHeight = (float)imgCrop.rows; //Give defualt value assuming vertex coordinates are in pixel units
    //cv::imwrite("textTex.png",imgCrop);

    data.mTexture.create(GL_ALPHA, cv::Size(imgCrop.cols, imgCrop.rows));
    data.mTexture.update(imgCrop);

    //Update tex coords
    for(int c = 0; c < TextFontData::kCharCount - 1; c++)
    {
        data.mCharTexStart[c] = (float)charOrigin[c] / imgCrop.cols;
        data.mCharTexEnd[c] = (float)charOrigin[c + 1] / imgCrop.cols;
    }
    data.mCharTexStart[TextFontData::kCharCount] = (float)charOrigin[TextFontData::kCharCount - 1] / imgCrop.cols;
    data.mCharTexEnd[TextFontData::kCharCount] = 1.0f;
}

void TextRenderer::renderText(std::stringstream &str)
{
	if(str.str().empty())
		return;

	std::vector<Eigen::Vector4f> vertexBuffer;
	std::vector<Eigen::Vector2f> texCoordsBuffer;

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    bool endsWithNewLine = (str.str()[str.str().length()-1] == '\n');

    std::string line;
    do
    {
        std::getline(str, line);

        int length = line.length();
		if (length > 0)
		{
			vertexBuffer.resize(6*length);
			texCoordsBuffer.resize(6*length);

			//Start
			for(std::size_t i = 0; i < line.length(); i++)
			{
				int c = line[i];
				float renderWidth = mActiveFont->mCharAspect[c] * mRenderCharHeight;

				Eigen::Vector4f *vertices = &vertexBuffer[i*6];
				Eigen::Vector2f *texCoords = &texCoordsBuffer[i * 6];

				vertices[0] = mRenderCaret;
				vertices[3] = vertices[1] = mRenderCaret + Eigen::Vector4f(0, mRenderCharHeight, 0, 0);
				vertices[4] = vertices[2] = mRenderCaret + Eigen::Vector4f(renderWidth, 0, 0, 0);
				vertices[5] =  mRenderCaret + Eigen::Vector4f(renderWidth, mRenderCharHeight, 0, 0);

				texCoords[0] = Eigen::Vector2f(mActiveFont->mCharTexStart[c], 0);
				texCoords[3] = texCoords[1] = Eigen::Vector2f(mActiveFont->mCharTexStart[c], 1.0f);
				texCoords[4] = texCoords[2] = Eigen::Vector2f(mActiveFont->mCharTexEnd[c], 0);
				texCoords[5] = Eigen::Vector2f(mActiveFont->mCharTexEnd[c], 1.0f);

				mRenderCaret[0] += renderWidth;
			}
			mShader->renderText(GL_TRIANGLES, mActiveFont->mTexture.getId(), vertexBuffer.data(), texCoordsBuffer.data(), 6*length, mActiveColor);
		}

        if(!str.eof() || endsWithNewLine)
        {
        	mRenderCaret[0] = mRenderCaret0[0];
        	mRenderCaret[1] += mRenderCharHeight;
        }
    }
    while(!str.eof());
}

}
