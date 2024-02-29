
#include "../rays/bsdf.h"
#include "../util/rand.h"
#include "debug.h"

namespace PT {

Vec3 reflect(Vec3 dir) {

    // TODO (PathTracer): Task 6
    // Return reflection of dir about the surface normal (0,1,0).
    Vec3 normal = Vec3(0, 1, 0);
    Vec3 reflected = dir - 2 * dot(dir, normal) * normal;
    return -reflected;
}

Vec3 refract(Vec3 out_dir, float index_of_refraction, bool& was_internal) {

    // TODO (PathTracer): Task 6
    // Use Snell's Law to refract out_dir through the surface
    // Return the refracted direction. Set was_internal to false if
    // refraction does not occur due to total internal reflection,
    // and true otherwise.

    // When dot(out_dir,normal=(0,1,0)) is positive, then out_dir corresponds to a
    // ray exiting the surface into vaccum (ior = 1). However, note that
    // you should actually treat this case as _entering_ the surface, because
    // you want to compute the 'input' direction that would cause this output,
    // and to do so you can simply find the direction that out_dir would refract
    // _to_, as refraction is symmetric.

    float eta = out_dir.y > 0 ? 1.f / index_of_refraction : index_of_refraction;

    if (abs(out_dir.y) < asin(1 / eta)) {
        was_internal = true;
        return {};
    }
    
    float cos_theta_i = sqrt(1 - (1 - pow(abs(out_dir.y), 2)) * pow(eta, 2));
    Vec3 in_dir {-eta * out_dir.x, -cos_theta_i * (out_dir.y / abs(out_dir.y)), -eta * out_dir.z};
    // assert(dot(in_dir, out_dir) < 0);
    return in_dir;
}

BSDF_Sample BSDF_Lambertian::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 5
    // Implement lambertian BSDF. Use of BSDF_Lambertian::sampler may be useful

    BSDF_Sample ret;
    ret.direction = sampler.sample(ret.pdf);
    ret.attenuation = evaluate(out_dir, ret.direction);

    return ret;
}

Spectrum BSDF_Lambertian::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    return albedo * (1.0f / PI_F);
}

BSDF_Sample BSDF_Mirror::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 6
    // Implement mirror BSDF

    BSDF_Sample ret;
    ret.attenuation = reflectance;
    ret.direction = reflect(out_dir);
    ret.pdf = 1.0f;
    return ret;
}

Spectrum BSDF_Mirror::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // Technically, we would return the proper reflectance
    // if in_dir was the perfectly reflected out_dir, but given
    // that we assume these are single exact directions in a
    // continuous space, just assume that we never hit them
    // _exactly_ and always return 0.
    return {};
}

BSDF_Sample BSDF_Glass::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 6

    // Implement glass BSDF.
    // (1) Compute Fresnel coefficient. Tip: use Schlick's approximation.
    // (2) Reflect or refract probabilistically based on Fresnel coefficient. Tip: RNG::coin_flip
    // (3) Compute attenuation based on reflectance or transmittance

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?

    // (1)
    float R0 = pow((1 - index_of_refraction) / (1 + index_of_refraction), 2);
    float fresnel_coef = R0 + (1 - R0) * pow((1 - abs(out_dir.y)), 5);
    
    // (2)
    if (RNG::coin_flip(fresnel_coef)) {
        // reflection
        BSDF_Sample ret;
        ret.direction = reflect(out_dir);
        ret.pdf = fresnel_coef;

        // (3)
        ret.attenuation = Spectrum(fresnel_coef / abs(ret.direction.y));
        return ret;
    } else {
        // refraction
        BSDF_Sample ret;
        ret.pdf = 0;

        BSDF_Refract refract {transmittance, index_of_refraction};
        while (ret.pdf == 0) {
            ret = refract.sample(out_dir);
            // total internal reflection if still 0, technically should not happen because we are using fresnel_coef
        }
        ret.pdf *= (1 - fresnel_coef);
        float eta = out_dir.y > 0 ? 1.f / index_of_refraction : index_of_refraction;

        // (3)
        ret.attenuation = Spectrum((1 - fresnel_coef) / abs(ret.direction.y) / pow(eta, 2));
        return ret;
    }
}

Spectrum BSDF_Glass::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // As with BSDF_Mirror, just assume that we never hit the correct
    // directions _exactly_ and always return 0.
    return {};
}

BSDF_Sample BSDF_Diffuse::sample(Vec3 out_dir) const {
    BSDF_Sample ret;
    ret.direction = sampler.sample(ret.pdf);
    ret.emissive = radiance;
    ret.attenuation = {};
    return ret;
}

Spectrum BSDF_Diffuse::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // No incoming light is reflected; only emitted
    return {};
}

BSDF_Sample BSDF_Refract::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 6
    // Implement pure refraction BSDF.

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?

    BSDF_Sample ret;
    ret.attenuation = transmittance; // What is the ratio of reflected/incoming light?
    bool was_internal = false;
    ret.direction = refract(out_dir, index_of_refraction, was_internal);       // What direction should we sample incoming light from?
    ret.pdf = was_internal ? 0.0f : 1.0f; // Was was the PDF of the sampled direction? (In this case, the PMF)
    return ret;
}

Spectrum BSDF_Refract::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // As with BSDF_Mirror, just assume that we never hit the correct
    // directions _exactly_ and always return 0.
    return {};
}

} // namespace PT
