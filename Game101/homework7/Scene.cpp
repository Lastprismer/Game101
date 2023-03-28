//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
	printf(" - Generating BVH...\n\n");
	this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

// 光线是否和场景中的某个包围盒相交
Intersection Scene::intersect(const Ray& ray) const {
	return this->bvh->Intersect(ray);
}

// 对场景中的光源进行随机采样
void Scene::sampleLight(Intersection& pos, float& pdf) const {
	// 发光区域面积
	float emit_area_sum = 0;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		if (objects[k]->hasEmit()) {
			emit_area_sum += objects[k]->getArea();
		}
	}
	// 在场景的所有光源上按面积随机采样一个点，并计算该点的概率密度
	float p = get_random_float() * emit_area_sum;
	emit_area_sum = 0;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		if (objects[k]->hasEmit()) {
			emit_area_sum += objects[k]->getArea();
			if (p <= emit_area_sum) {
				// 对光源采样，得到位置和pdf
				objects[k]->Sample(pos, pdf);
				break;
			}
		}
	}
}

bool Scene::trace(
	const Ray& ray,
	const std::vector<Object*>& objects,
	float& tNear, uint32_t& index, Object** hitObject) {
	*hitObject = nullptr;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		float tNearK = kInfinity;
		uint32_t indexK;
		Vector2f uvK;
		if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
			*hitObject = objects[k];
			tNear = tNearK;
			index = indexK;
		}
	}


	return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray& ray, int depth) const {
	/*

	shade(p, wo)
		sampleLight(inter , pdf_light)
		Get x, ws, NN, emit from inter
		Shoot a ray from p to x
		If the ray is not blocked in the middle
		L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws, NN) / |x-p|^2 / pdf_light

		L_indir = 0.0
		Test Russian Roulette with probability RussianRoulette
		wi = sample(wo, N)
		Trace a ray r(p, wi)
		If ray r hit a non -emitting object at q
		L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N) / pdf(wo, wi, N) / RussianRoulette

		Return L_dir + L_indir

	*/
	const float EPS = 0.0016f;

	Vector3f L_dir;
	Vector3f L_indir;

	/* 什么都没打到 */
	Intersection obj_intersection = intersect(ray);
	if (!obj_intersection.happened)
		return L_dir;

	/* 打到光源 */
	if (obj_intersection.m->hasEmission())
		return obj_intersection.m->getEmission();

	/* 打到非光源物体 */
	Vector3f p = obj_intersection.coords;
	auto obj_m = obj_intersection.m;
	Vector3f N = obj_intersection.normal;
	/* wo定义与课程介绍（从物体到眼睛）相反（从眼睛到物体） */
	Vector3f wo = ray.direction;

	// sampleLight(inter, pdf_light)
	float pdf_light;
	Intersection light_intersection;
	sampleLight(light_intersection, pdf_light);

	// Get x, ws, NN, emit from inter
	Vector3f x = light_intersection.coords;
	Vector3f ws = (x - p).normalized();
	Vector3f NN = light_intersection.normal;
	Vector3f emit = light_intersection.emit;

	// Shoot a ray from p to x
	Ray obj_to_light(p, ws);

	// If the ray is not blocked in the middle
	float d1 = intersect(obj_to_light).distance;
	float d2 = (x - p).norm();
	if (d2 - d1 <= EPS) {
		// L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws, NN) / |x - p | ^ 2 / pdf_light
		/* ws计算cos_theta_x时取负：ws是物体指向光源，与NN夹角大于180度 */
		L_dir = emit * obj_m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / std::pow(d2, 2) / pdf_light;
	}

	// L_indir = 0.0
	// Test Russian Roulette with probability RussianRoulette
	float P_RR = get_random_float();
	if (P_RR < RussianRoulette) {
		// wi = sample(wo, N)
		Vector3f wi = obj_m->sample(wo, N).normalized();
		// Trace a ray r(p, wi)
		Ray r(p, wi);
		// If ray r hit a non - emitting object at q
		Intersection inter = intersect(r);
		if (inter.happened && !inter.m->hasEmission()) {
			// L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N) / pdf(wo, wi, N) / RussianRoulette
			float pdf = obj_m->pdf(wo, wi, N);
			if (pdf > EPS)
				L_indir = castRay(r, depth + 1) * obj_m->eval(wo, wi, N) * dotProduct(wi, N) / pdf / RussianRoulette;
		}
	}

	// Return L_dir + L_indir
	return L_dir + L_indir;
		
}