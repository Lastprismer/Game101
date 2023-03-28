#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
	SplitMethod splitMethod)
	: maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
	primitives(std::move(p)) {
	time_t start, stop;
	time(&start);
	if (primitives.empty())
		return;

	root = splitMethod == SplitMethod::NAIVE ? recursiveBuild(primitives) : recursiveBuildSAH(primitives);

	time(&stop);
	double diff = difftime(stop, start);
	int hrs = (int)diff / 3600;
	int mins = ((int)diff / 60) - (hrs * 60);
	int secs = (int)diff - (hrs * 3600) - (mins * 60);

	printf("Mode: %s\n", splitMethod == SplitMethod::NAIVE ? "Naive" : "SAH");
	printf(
		"\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
		hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects) {
	BVHBuildNode* node = new BVHBuildNode();

	// Compute bounds of all primitives in BVH node
	Bounds3 bounds;
	for (int i = 0; i < objects.size(); ++i)
		bounds = Union(bounds, objects[i]->getBounds());
	if (objects.size() == 1) {
		// Create leaf _BVHBuildNode_
		node->bounds = objects[0]->getBounds();
		node->object = objects[0];
		node->left = nullptr;
		node->right = nullptr;
		return node;
	}
	else if (objects.size() == 2) {
		node->left = recursiveBuild(std::vector{ objects[0] });
		node->right = recursiveBuild(std::vector{ objects[1] });

		node->bounds = Union(node->left->bounds, node->right->bounds);
		return node;
	}
	else {
		Bounds3 centroidBounds;
		for (int i = 0; i < objects.size(); ++i)
			centroidBounds =
			Union(centroidBounds, objects[i]->getBounds().Centroid());
		int dim = centroidBounds.maxExtent();
		switch (dim) {
		case 0:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
				{
					return f1->getBounds().Centroid().x <
						f2->getBounds().Centroid().x;
				});
			break;
		case 1:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
				{
					return f1->getBounds().Centroid().y <
						f2->getBounds().Centroid().y;
				});
			break;
		case 2:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
				{
					return f1->getBounds().Centroid().z <
						f2->getBounds().Centroid().z;
				});
			break;
		}
		auto beginning = objects.begin();
		auto middling = objects.begin() + (objects.size() / 2);
		auto ending = objects.end();

		auto leftshapes = std::vector<Object*>(beginning, middling);
		auto rightshapes = std::vector<Object*>(middling, ending);

		assert(objects.size() == (leftshapes.size() + rightshapes.size()));

		node->left = recursiveBuild(leftshapes);
		node->right = recursiveBuild(rightshapes);

		node->bounds = Union(node->left->bounds, node->right->bounds);
	}

	return node;
}

BVHBuildNode* BVHAccel::recursiveBuildSAH(std::vector<Object*>objects) {
	BVHBuildNode* node = new BVHBuildNode();

	// Compute bounds of all primitives in BVH node
	Bounds3 bounds;
	for (int i = 0; i < objects.size(); ++i)
		bounds = Union(bounds, objects[i]->getBounds());
	if (objects.size() == 1) {
		// Create leaf _BVHBuildNode_
		node->bounds = objects[0]->getBounds();
		node->object = objects[0];
		node->left = nullptr;
		node->right = nullptr;
		return node;
	}
	else if (objects.size() == 2) {
		node->left = recursiveBuild(std::vector{ objects[0] });
		node->right = recursiveBuild(std::vector{ objects[1] });

		node->bounds = Union(node->left->bounds, node->right->bounds);
		return node;
	}
	else {
		Bounds3 centroidBounds;
		for (int i = 0; i < objects.size(); ++i)
			centroidBounds =
			Union(centroidBounds, objects[i]->getBounds().Centroid());
		int dim = centroidBounds.maxExtent();
		switch (dim) {
		case 0:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
				{
					return f1->getBounds().Centroid().x <
						f2->getBounds().Centroid().x;
				});
			break;
		case 1:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
				{
					return f1->getBounds().Centroid().y <
						f2->getBounds().Centroid().y;
				});
			break;
		case 2:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
				{
					return f1->getBounds().Centroid().z <
						f2->getBounds().Centroid().z;
				});
			break;
		}
		auto beginning = objects.begin();
		auto middling = objects.begin() + (objects.size() / 2);
		auto ending = objects.end();

		// https://blog.csdn.net/kuangben2000/article/details/102419317
		// ±éÀú·Öµã, Í°
		int part = 10;
		int size = objects.size();
		double min_weigh = 0x3f3f3f3f;
		double boundArea = bounds.SurfaceArea();
		int cut = 0;

		for (int i = 1; i < part; i++) {
			middling = objects.begin() + size * i / part;
			int a = size * i / part;
			auto leftshapes = std::vector<Object*>(beginning, middling);
			auto rightshapes = std::vector<Object*>(middling, ending);

			assert(size == (leftshapes.size() + rightshapes.size()));

			Bounds3 bound1, bound2;
			for (Object* obj : leftshapes) {
				bound1 = Union(bound1, obj->getBounds());
			}
			for (Object* obj : rightshapes) {
				bound2 = Union(bound2, obj->getBounds());
			}

			// SAH
			double bound1Area = bound1.SurfaceArea(),
				bound2Area = bound2.SurfaceArea();

			/*
			cost(A,B) = p(A) * ¡Æt(i) in A  + p(B) * ¡Æt(j) in B
					  = p(A) * n in A  + p(B) * m in B
			*/

			double weigh = (bound1Area / boundArea * leftshapes.size() + bound2Area / boundArea * rightshapes.size());
			if (weigh < min_weigh) {
				min_weigh = weigh;
				cut = i;
			}
		}
		middling = objects.begin() + size * cut / part;

		auto leftshapes = std::vector<Object*>(beginning, middling);
		auto rightshapes = std::vector<Object*>(middling, ending);

		node->left = recursiveBuildSAH(leftshapes);
		node->right = recursiveBuildSAH(rightshapes);

		node->bounds = Union(node->left->bounds, node->right->bounds);

	}
	return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const {
	Intersection isect;
	if (!root)
		return isect;
	isect = BVHAccel::getIntersection(root, ray);
	return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const {
	// TODO Traverse the BVH to find intersection

	Intersection inter;
	std::array<int, 3> dirIsNeg = { ray.direction.x < 0, ray.direction.y < 0, ray.direction.z < 0 };

	if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
		return inter;
	if (node->left == nullptr && node->right == nullptr)
		return node->object->getIntersection(ray);

	Intersection hit1 = getIntersection(node->left, ray);
	Intersection hit2 = getIntersection(node->right, ray);
	return hit1.distance < hit2.distance ? hit1 : hit2;
}