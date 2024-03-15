
#include "../rays/pathtracer.h"
#include "../rays/samplers.h"
#include "../util/rand.h"
#include "debug.h"

namespace PT {

// Return the radiance along a ray entering the camera and landing on a
// point within pixel (x,y) of the output image.
//
Spectrum Pathtracer::trace_pixel(size_t x, size_t y) {

    Vec2 xy((float)x, (float)y);
    Vec2 wh((float)out_w, (float)out_h);

    // TODO (PathTracer): Task 1

    // Generate a sample within the pixel with coordinates xy and return the
    // incoming light using trace_ray.

    // If n_samples is 1, please send the ray through the center of the pixel.
    // If n_samples > 1, please send the ray through any random point within the pixel

    // Tip: consider making a call to Samplers::Rect::Uniform

    // Tip: you may want to use log_ray for debugging. Given ray t, the following lines
    // of code will log .03% of all rays (see util/rand.h) for visualization in the app.
    // see student/debug.h for more detail.

    // As an example, the code below generates a ray through the bottom left of the
    // specified pixel

    // Step 1: Compute normalized screen space point
    Vec2 sample_xy; // pixel space coordinates of sample points
    if(n_samples == 1) {
        sample_xy = xy + Vec2{.5f, .5f};
    } else {
        float pdf = 0;
        auto rect_uni = Samplers::Rect::Uniform(Vec2{1.f, 1.f});
        sample_xy = xy + rect_uni.sample(pdf);
    }

    // Step 2:
    Ray out = camera.generate_ray(sample_xy / wh);

    if (RNG::coin_flip(0.0003f)) log_ray(out, 10.0f);

    return trace_ray(out);
}

Vec3 sample_phase_function(const Vec3& out_dir, float& pdf) {
    // Uniform sphere sampling
    auto sampler = Samplers::Sphere::Uniform();
    Vec3 sample = sampler.sample(pdf);
    return sample;
}

Spectrum eval_phase_function(const Vec3& in_dir, const Vec3& out_dir) {
    return Spectrum(1.f / (4 * PI_F));
}

Spectrum Pathtracer::trace_ray(const Ray& ray) {
    int current_medium = 1; // TODO: implement this
    Spectrum radiance_out;
    Ray in_ray = ray;
    in_ray.depth = 0;

    while (in_ray.depth <= max_depth) {
        // Trace ray into scene. If nothing is hit, might be in a volume,
        // If not in volume, sample the environment 
        Trace hit = scene.hit(in_ray);
        if(!hit.hit && current_medium == 0) {
            if(env_light.has_value()) {
                radiance_out += env_light.value().sample_direction(in_ray.dir);
            }
            break;
        }

        // Get parameters for volume renderin
        Spectrum absorption(absorb);
        Spectrum scattering(scatter);
        Spectrum extinction = absorption + scattering;
        float u = RNG::unit();
        float t = -std::log(1 - u) / extinction.to_vec().mean();  // FIXME: should we use mean here?
        Spectrum vol_transmittance = (-1.f * extinction * t).exp();
        Spectrum trans_pdf = (-1.f * extinction * t).exp() * extinction;
        bool scatter = t < hit.distance;
        if (!scatter) t = hit.distance;

        if (!volume_rendering) {
            if (!hit.hit) break;
            absorption = Spectrum(0);
            scattering = Spectrum(0);
            extinction = Spectrum(0);
            t = hit.distance;
            scatter = false;
            vol_transmittance = Spectrum(1);
            trans_pdf = Spectrum(1);
        }

        in_ray.throughput *= vol_transmittance / trans_pdf;

        Spectrum Lo = Spectrum(0.0f);

        // lambda function to sample a light. Called in loop below.
        auto sample_light = [&](const auto& light, const BSDF* bsdf, const Vec3& out_dir, const Mat4& world_to_object) {
            // If the light is discrete (e.g. a point light), then we only need
            // one sample, as all samples will be equivalent
            int samples = light.is_discrete() ? 1 : (int)n_area_samples;
            for(int i = 0; i < samples; i++) {

                // Grab a sample of the light source. See rays/light.h for definition of this
                // struct. Most importantly for Task 4, it contains the distance to the light from
                // hit.position.
                Light_Sample sample = light.sample(hit.position);
                Vec3 in_dir = world_to_object.rotate(sample.direction);

                // Construct a shadow ray and compute whether the intersected surface is
                // in shadow. Only accumulate light if not in shadow.
                Vec3 shadow_origin = hit.position;
                Vec3 shadow_dir = sample.direction;
                Ray shadow_ray(shadow_origin, shadow_dir);
                shadow_ray.dist_bounds.x = EPS_F;
                shadow_ray.dist_bounds.y = sample.distance - EPS_F;
                Trace shadow_hit = scene.hit(shadow_ray);
                if(shadow_hit.hit) continue;

                Spectrum attenuation;
                if (bsdf) {
                    // If the light is below the horizon, ignore it
                    float cos_theta = in_dir.y;
                    if(cos_theta <= 0.0f) continue;
                    attenuation = cos_theta * bsdf->evaluate(out_dir, in_dir);
                } else {
                    attenuation = (-1.f * extinction * sample.distance).exp(); // transmittance from the light source
                }
                if(attenuation.luma() == 0.0f) continue;

                // Note: that along with the typical cos_theta, pdf factors, we divide by samples.
                // This is because we're doing another monte-carlo estimate of the lighting from
                // area lights here.
                Lo += (1 / (samples * sample.pdf)) * sample.radiance * attenuation;
            }
        };
        

        Vec3 in_dir;
        if (scatter) {
            // hit the volume, compute a in-scattering direction, modify ray, and keep recursing
            Vec3 out_dir = (in_ray.point - hit.position).unit(); // This is in object space

            // sample indirect lighting incoming ray in object space
            float pdf = 0;
            in_dir = sample_phase_function(out_dir, pdf);
            in_ray.throughput *= eval_phase_function(in_dir, out_dir) / pdf * scattering;
            
            // loop over all the lights and accumulate radiance.
            // for(const auto& light : lights) sample_light(light, nullptr);
            // if(env_light.has_value()) sample_light(env_light.value(), nullptr);

            in_ray.from_volume = true;
            in_ray.point = in_ray.point + t * in_ray.dir;
            in_ray.dir = in_dir;  // FIXME: which in_dir should we use, i.e. scatter or not?
            in_ray.depth++;
            in_ray.dist_bounds.x = EPS_F;
            in_ray.dist_bounds.y = std::numeric_limits<float>::max();
        } else {
            // hit a surface, compute direct and indirect lighting

            // Set up a coordinate frame at the hit point, where the surface normal becomes {0, 1, 0}
            // This gives us out_dir and later in_dir in object space, where computations involving the
            // normal become much easier. For example, cos(theta) = dot(N,dir) = dir.y!
            Mat4 object_to_world = Mat4::rotate_to(hit.normal);
            Mat4 world_to_object = object_to_world.T();
            Vec3 out_dir = world_to_object.rotate(in_ray.point - hit.position).unit(); // This is in object space

            // If we're using a two-sided material, treat back-faces the same as front-faces
            const BSDF& bsdf = materials[hit.material];
            if(!bsdf.is_sided() && dot(hit.normal, in_ray.dir) > 0.0f) {
                hit.normal = -hit.normal;
            }

            // sample indirect lighting incoming ray in object space
            BSDF_Sample bsdf_sample = bsdf.sample(out_dir); 
            in_dir = bsdf_sample.direction;

            // If the BSDF is discrete (i.e. uses dirac deltas/if statements), then we are never
            // going to hit the exact right direction by sampling lights, so ignore them.
            if(!bsdf.is_discrete()) {
                // loop over all the lights and accumulate radiance.
                for(const auto& light : lights) sample_light(light, &bsdf, out_dir, world_to_object);
                if(env_light.has_value()) sample_light(env_light.value(), &bsdf, out_dir, world_to_object);
            }

            // account first point emission, specular reflection, and volume 
            if(in_ray.depth == 0 || in_ray.from_discrete || in_ray.from_volume) { 
                Lo += bsdf_sample.emissive;
            }

            radiance_out += in_ray.throughput * Lo;

            // update throughput with bsdf attenuation
            float cos_theta = abs(in_dir.y);
            in_ray.throughput *= bsdf_sample.attenuation * cos_theta / bsdf_sample.pdf;
    
            //TODO: handle hitting index-matching medium: update medium
            // TODO: handle updating current medium: when we hit a surface that changes medium

            // compute next ray in world space
            in_ray.from_discrete = bsdf.is_discrete();
            in_ray.from_volume = false;
            in_ray.point = in_ray.point + t * in_ray.dir;
            in_ray.dir = object_to_world.rotate(in_dir);  // FIXME: which in_dir should we use, i.e. scatter or not?
            in_ray.depth++;
            in_ray.dist_bounds.x = EPS_F;
            in_ray.dist_bounds.y = std::numeric_limits<float>::max();
        }


        // Debugging: if the normal colors flag is set, return the normal color
        // if(debug_data.normal_colors) return Spectrum::direction(hit.normal);

        // Russian Roulette
        float terminate_prob = 1.0f - std::max(in_ray.throughput.luma(), 0.05f);
        if(RNG::coin_flip(terminate_prob)) {
            break;
        }

        in_ray.throughput *= 1 / (1.0f - terminate_prob);
    }

    return radiance_out;
}

void Pathtracer::set_volume_rendering(bool vr) {
    volume_rendering = vr;
}

void Pathtracer::set_absorb_and_scatter(float _abs, float _sct) {
    absorb = _abs;
    scatter = _sct;
}

} // namespace PT
