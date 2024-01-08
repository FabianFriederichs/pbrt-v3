#include "tests/gtest/gtest.h"
#include "geometry.h"
#include "pbrt.h"
#include <random>
#include <vector>

using namespace pbrt;

struct ANRIntersectionTest : testing::Test
{
    constexpr static std::size_t c_seed = 12345;
    constexpr static std::size_t c_num_pairs = 1000000;    
    constexpr static Float c_epsilon = 1e-5f;
    constexpr static Float c_aabb_size_min = 0.001f;
    constexpr static Float c_aabb_size_max = 1.5f;
    constexpr static Float c_origin_min = -1.0f;
    constexpr static Float c_origin_max = 1.0f;

    std::size_t m_num_pairs;
    std::vector<Ray> m_rays;
    std::vector<Bounds3f> m_aabbs;
	
    ANRIntersectionTest() : 
        m_num_pairs(c_num_pairs),
        m_rays(), 
        m_aabbs()
    {
        std::default_random_engine rng(c_seed);
        std::uniform_real_distribution<Float> sample_ray_origin(c_origin_min, c_origin_max);
        std::uniform_real_distribution<Float> sample_ray_direction(-1.0f, 1.0f);
        std::uniform_real_distribution<Float> sample_aabb_origin(c_origin_min, c_origin_max);
        std::uniform_real_distribution<Float> sample_aabb_size(c_aabb_size_min, c_aabb_size_max);

        m_rays.reserve(c_num_pairs);
        m_aabbs.reserve(c_num_pairs);

		// generate random rays with origin in [-1, 1]
        for (std::size_t r = 0; r < c_num_pairs; ++r)
        {
            // sample origin
            Point3f o{ sample_ray_origin(rng), sample_ray_origin(rng), sample_ray_origin(rng) };
            // sample direction
            Vector3f d { sample_ray_direction(rng), sample_ray_direction(rng), sample_ray_direction(rng) };
            // normalize direction
            d /= d.Length();
            // add ray
            m_rays.push_back(Ray(o, d));
        }

		// generate random AABBs with center in [-1, 1] and size in [0.1, 1.9]
        for (std::size_t b = 0; b < c_num_pairs; ++b)
        {
            // sample min
            Point3f bmin { sample_aabb_origin(rng), sample_aabb_origin(rng), sample_aabb_origin(rng) };
            // sample size
            Vector3f aabb_size { sample_aabb_size(rng), sample_aabb_size(rng), sample_aabb_size(rng) };
            // add aabb
            m_aabbs.push_back(Bounds3f(bmin, bmin + aabb_size));
        }
    }
};

TEST_F(ANRIntersectionTest, AxisNormalizedRayAABBIntersectP) {
    for (std::size_t i = 0; i < m_num_pairs; ++i)
    {
        // fetch data
        const auto& aabb = m_aabbs[i];
        const auto& ray = m_rays[i];

        const auto nr = normalizeRay(ray);

        // do intersections
        const Vector3f invDir { 1.0f / ray.d[0], 1.0f / ray.d[1], 1.0f / ray.d[2] };
        const int dirIsNeg[] = { ray.d[0] < 0.0f, ray.d[1] < 0.0f, ray.d[2] < 0.0f };
        const bool intersect_res = aabb.IntersectP(ray, invDir, dirIsNeg);
        const bool intersect_anr_res = aabb.IntersectNRP(nr.ray, nr.invDir, nr.rayClass, nr.dirIsNeg);

        // check results
        EXPECT_EQ(intersect_anr_res, intersect_res);
    }
}

TEST_F(ANRIntersectionTest, AxisNormalizedRayAABBIntersect) {
    for (std::size_t i = 0; i < m_num_pairs; ++i)
    {
        // fetch data
        const auto& aabb = m_aabbs[i];
        const auto& ray = m_rays[i];

        const auto nr = normalizeRay(ray);

        // do intersections
        Float tNear;
        Float tFar;
        Float tNear_anr;
        Float tFar_anr;
        const bool intersect_res = aabb.IntersectP(ray, &tNear, &tFar);
        const bool intersect_anr_res = aabb.IntersectNRP(nr.ray, nr.rayClass, &tNear_anr, &tFar_anr);
        // convert intersection distances to anr ray
        tNear = ray.o[nr.dominantAxis] + tNear * ray.d[nr.dominantAxis];
        tFar = ray.o[nr.dominantAxis] + tFar * ray.d[nr.dominantAxis];
        // if dominant axis is negative, swap tNear and tFar
        if (isNegative(nr.rayClass))
            std::swap(tNear, tFar);

        // check results
        EXPECT_EQ(intersect_anr_res, intersect_res);
        if (intersect_res)
        {
            EXPECT_NEAR(tNear_anr, tNear, c_epsilon);
            EXPECT_NEAR(tFar_anr, tFar, c_epsilon);
        }
    }
}
