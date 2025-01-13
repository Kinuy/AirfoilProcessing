# pragma once

class Node {
public:
	Node(int index, double x, double y, bool isTop, bool isLastFlatbackPoint);
	double X() { return mX; }
	double Y() { return mY; }
	bool IsTop() { return mIsTop; }
	bool IsLastFlatbackPoint() { return mIsLastFlatbackPoint; }
	int Index() { return mIndex; }
private:
	int mIndex;
	double mX;
	double mY;
	bool mIsLastFlatbackPoint;
	bool mIsTop;
	bool mTopBottom;
	enum mSide { ps, ss };

};
