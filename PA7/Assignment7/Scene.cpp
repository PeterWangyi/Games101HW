//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
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
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    if (depth > 2)
        return Vector3f();
    Intersection interToScene = Scene::intersect(ray);
    if (!interToScene.happened){
        return Vector3f();
    }
    // Light source itself
    if (interToScene.m->hasEmission()){
        return interToScene.m->getEmission();
    }

    Vector3f L_dir = {0, 0, 0}, L_indir = {0, 0, 0};

    //Calculate the Intersection from point to light in order to calculate direct Color
    Intersection LightPos;
    float lightpdf;
    sampleLight(LightPos, lightpdf);
    Vector3f LightDir = LightPos.coords - interToScene.coords;
    float dis = dotProduct(LightDir, LightDir);
    Vector3f LightDirNormal = LightDir.normalized();
    Ray rayToLight(interToScene.coords, LightDirNormal);
    Intersection interToLight = Scene::intersect(rayToLight);
    auto f_r = interToScene.m->eval(ray.direction, LightDirNormal, interToScene.normal);
    if (interToLight.distance - LightDir.norm() > -0.005){
        L_dir = LightPos.emit * f_r * dotProduct(LightDirNormal, interToScene.normal) * dotProduct(-LightDirNormal, LightPos.normal) / dis / lightpdf;
    }

    //Calculate the Intersection from point to point in order to calculate indirect Color
    if(get_random_float()>RussianRoulette)
        return L_dir;

    Vector3f wi =interToScene.m->sample(ray.direction,interToScene.normal).normalized();
    //Ray indirRay = Ray(intersToScene.coords, wi);
    Ray indirRay(interToScene.coords, wi);
    Intersection intersToPoint = Scene::intersect(indirRay);
    if( intersToPoint.happened && !intersToPoint.m->hasEmission())
    {
        float pdf=interToScene.m->pdf(ray.direction,wi,interToScene.normal);
        L_indir= castRay(indirRay,depth+1) * interToScene.m->eval(ray.direction,wi,interToScene.normal) * dotProduct(wi,interToScene.normal) /(RussianRoulette/pdf);
    }

    return L_dir+ L_indir ;


}