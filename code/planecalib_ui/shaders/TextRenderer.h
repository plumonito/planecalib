/*
 * TextRenderer.h
 *
 *  Created on: May 10, 2013
 *      Author: danielh
 */

#ifndef TEXTRENDERER_H_
#define TEXTRENDERER_H_

#include <Eigen/Dense>
#include <sstream>
#include <memory>
#include "../TextureHelper.h"
#include "TextShader.h"

namespace planecalib
{

class TextRenderer;

class TextRendererStream
{
public:
	TextRendererStream(TextRenderer &renderer): mRenderer(renderer)
	{
	}

	~TextRendererStream();

	void flush();

	void setColor(const Eigen::Vector4f &color);

    template<class T>
    TextRendererStream &operator <<(const T &value);
	
protected:
    TextRenderer &mRenderer;
    std::stringstream mStream;
};

class TextRenderer
{
public:
    TextRenderer();
    ~TextRenderer();

    /**
     * @brief Builds the texture used for text rendering.
     */
    bool init(TextShader *shader);

    void setActiveFontSmall() {mActiveFont = &mFontData[0];}
    void setActiveFontBig() {mActiveFont = &mFontData[1];}

	void setMVPMatrix(const Eigen::Matrix4f &mvp) { mShader->setMVPMatrix(mvp); }
    void setRenderCharHeight(float height) {mRenderCharHeight = height;}
    void setCaret(const Eigen::Vector4f &caret) {mRenderCaret = mRenderCaret0 = caret;}
    void setCaret(const Eigen::Vector2f &caret) {setCaret(Eigen::Vector4f(caret.x(), caret.y(), 1, 1));}
    void setColor(const Eigen::Vector4f &color) {mActiveColor = color;}

    void renderText(const std::string &str)
    {
        std::stringstream ss(str);
        renderText(ss);
    }
    void renderText(std::stringstream &str);

protected:
    class TextFontData
    {
    public:
        static const int kCharCount = 255;

        TextureHelper mTexture;
        float mCharAspect[kCharCount];
        float mCharTexStart[kCharCount+1];
        float mCharTexEnd[kCharCount+1];
    };

    TextShader *mShader;

    std::vector<TextFontData> mFontData;
    TextFontData *mActiveFont;
    Eigen::Vector4f mActiveColor;

    float mRenderCharHeight;

    Eigen::Vector4f mRenderCaret0;
    Eigen::Vector4f mRenderCaret;

    void prepareFontData(TextFontData &data, int face, double scale, int thickness, int lineType);
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Implementation
template<class T>
TextRendererStream &TextRendererStream::operator <<(const T &value)
{
	mStream << value;
	return *this;
}

}

#endif /* TEXTRENDERER_H_ */
