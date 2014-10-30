/*
 * GLImageTiler.cpp
 *
 *  Created on: Feb 15, 2013
 *      Author: danielh
 */

#include "ViewportTiler.h"
#include <cassert>
#include "GL/glew.h"

namespace planecalib
{

ViewportTiler::ViewportTiler()
        : mRows(0), mCols(0)
{

}

void ViewportTiler::configDevice(const Eigen::Vector2i &viewportOrigin, const Eigen::Vector2i &viewportSize, int rows, int cols)
{
	assert(rows>0 && cols>0);
	mRows = rows;
	mCols = cols;

	mViewportOrigin = viewportOrigin;
	mViewportSize = viewportSize;
	mCellSize = Eigen::Vector2i(viewportSize.x()/cols, viewportSize.y()/rows);

	mFullScreenTile.mAspectRatio = (float)mViewportSize.x() / mViewportSize.y();
	mFullScreenTile.mViewportOrigin = mViewportOrigin;
	mFullScreenTile.mViewportSize = mViewportSize;
	setImageMVP(&mFullScreenTile, mViewportSize);

	mTiles.clear();
	mTileMatrix.clear();
	mTileMatrix.resize(rows*cols, NULL);
}

void ViewportTiler::addTile(int row, int col, float aspectRatio, int rowSpan, int colSpan)
{
	ViewportTileInfo *tile = new ViewportTileInfo();
	mTiles.push_back(std::unique_ptr<ViewportTileInfo>(tile));

	tile->mViewportOrigin.x() = mViewportOrigin.x() + col*mCellSize.x();
	tile->mViewportOrigin.y() = mViewportOrigin.y() + row*mCellSize.y();
	tile->mViewportSize.x() = mCellSize.x()*colSpan;
	tile->mViewportSize.y() = mCellSize.y()*rowSpan;

	tile->mAspectRatio = aspectRatio;
	resetMVP(tile);

	int endRow=row+rowSpan;
	int endCol=col+colSpan;
	for(int j=row; j<endRow; j++)
	{
		for(int i=col; i<endCol; i++)
		{
			ViewportTileInfo *&cell = mTileMatrix[j*mCols + i];
			assert(cell==NULL);
			cell = tile;
		}
	}
}

void ViewportTiler::resetMVP(ViewportTileInfo *tile)
{
	//Configure mvp matrix to correct aspect ratio
	float viewportAspect = static_cast<float>(tile->mViewportSize.x()) / tile->mViewportSize.x();
	if(tile->mAspectRatio == -1.0f)
	{
		tile->mAspectRatio = viewportAspect;
		tile->mMvp = Eigen::Matrix4f::Identity();
	}
	else
	{
		tile->mMvp = createAspectMVP(viewportAspect, tile->mAspectRatio);
	}
}

Eigen::Matrix4f ViewportTiler::createAspectMVP(float viewportAspect, float newAspect)
{
	if(viewportAspect > newAspect)
	{
		//Shrink in x
		return (Eigen::Matrix4f() << newAspect / viewportAspect, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1).finished();
	}
	else
	{
		//Shrink in y
		return (Eigen::Matrix4f() << 1, 0, 0, 0, 0, viewportAspect / newAspect, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1).finished();
	}
}

void ViewportTiler::setImageMVP(ViewportTileInfo *tile, const Eigen::Vector2i &imageSize)
{
	float viewportAspect = static_cast<float>(tile->mViewportSize.x()) / tile->mViewportSize.y();
	tile->mMvp = GetImageSpaceMvp(viewportAspect, imageSize);
}


Eigen::Matrix4f ViewportTiler::GetImageSpaceMvp(float viewportAspect, const Eigen::Vector2i &imageSize)
{
	float imageAspect = static_cast<float>(imageSize.x()) / imageSize.y();
	Eigen::Vector2f paddedSize = imageSize.cast<float>();
    if (viewportAspect > imageAspect)
    {
        // Viewport is wider than image
        // Use entire viewport height and add padding on x
    	paddedSize.x() += viewportAspect * imageSize.y() - imageSize.x();
    }
    else
    {
        // Viewport is taller than rectangle
        // Use entire viewport width and add padding on y
    	paddedSize.y() += imageSize.x() / viewportAspect - imageSize.y();
    }

    float zNear = 0.001f;
    float zFar = 100.0f;

    const float sx = (2.0f) / (paddedSize.x());
	const float tx = -((imageSize.x()) / 2.0f) * sx;
    //const float tx = -1;
	const float sy = -(2.0f) / (paddedSize.y());
	const float ty = -((imageSize.y()) / 2.0f) * sy;
    //const float ty = 1;
    const float sz = -2.0f / (zFar - zNear);
    const float tz  = -2.0f / (zFar + zNear);

    return (Eigen::Matrix4f() << sx,0,tx,0, 0,sy,ty,0, 0,0,sz,tz, 0,0,1,0).finished();
}

void ViewportTiler::fillTiles()
{
	for(int j=0; j<mRows; j++)
	{
		for(int i=0; i<mCols; i++)
		{
			ViewportTileInfo *&cell = mTileMatrix[j*mCols + i];
			if(cell==NULL)
				addTile(j,i);
		}
	}
}

void ViewportTiler::setActiveTile(ViewportTileInfo *tile)
{
	mActiveTile = tile;
	glViewport(mActiveTile->mViewportOrigin.x(), mActiveTile->mViewportOrigin.y(), mActiveTile->mViewportSize.x(), mActiveTile->mViewportSize.y());
}

void ViewportTiler::screenToVertex(const Eigen::Vector2f &screenPoint, int &tileIdx, Eigen::Vector4f &vertex) const
{
	int tileX = (int)(screenPoint.x() / mCellSize.x());
	int tileY = (int)(screenPoint.y() / mCellSize.y());

	if(tileX < 0 || tileX >= mCols || tileY < 0 || tileY >= mRows)
	{
		tileIdx = -1;
		return;
	}

	tileIdx = tileY*mCols + tileX;
	const auto &tile = *mTiles[tileIdx];
	Eigen::Matrix4f mvpInv = tile.mMvp.inverse();
	vertex = mvpInv * Eigen::Vector4f(2 * (screenPoint.x() - tile.mViewportOrigin.x()) / tile.mViewportSize.x() - 1, 1 - 2 * (screenPoint.y() - tile.mViewportOrigin.y()) / tile.mViewportSize.y(), 1, 1);
}

}
