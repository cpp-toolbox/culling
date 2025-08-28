#ifndef CULLING_HPP
#define CULLING_HPP

#include <glm/glm.hpp>
#include <array>

#include "sbpt_generated_includes.hpp"

class ICuller {
  public:
    virtual ~ICuller() = default;
    virtual bool is_visible(const std::vector<glm::vec3> &xyz_positions, Transform &transform) = 0;
};

// NOTE: I also want to make a version of this or something that is designed ot work on orthographic top down type
// things as well
class FrustumCuller : public ICuller {
  public:
    explicit FrustumCuller(FPSCamera &cam, const unsigned int &screen_width_px, const unsigned int &screen_height_px)
        : camera(cam), screen_width_px(screen_width_px), screen_height_px(screen_height_px) {
        update_frustum_planes();
    }

    void update_frustum_planes() {
        // Camera params
        float aspect = static_cast<float>(screen_width_px) / screen_height_px;
        float fov_y = glm::radians(camera.fov.get());
        float near_z = camera.near_plane;
        float far_z = camera.far_plane;

        float tan_half_fov_y = tanf(fov_y * 0.5f);
        float near_height = 2.0f * near_z * tan_half_fov_y;
        float near_width = near_height * aspect;
        float far_height = 2.0f * far_z * tan_half_fov_y;
        float far_width = far_height * aspect;

        // Camera-space corners
        std::array<glm::vec3, 8> cam_corners = {
            glm::vec3(-near_width / 2, -near_height / 2, -near_z), glm::vec3(near_width / 2, -near_height / 2, -near_z),
            glm::vec3(-near_width / 2, near_height / 2, -near_z),  glm::vec3(near_width / 2, near_height / 2, -near_z),
            glm::vec3(-far_width / 2, -far_height / 2, -far_z),    glm::vec3(far_width / 2, -far_height / 2, -far_z),
            glm::vec3(-far_width / 2, far_height / 2, -far_z),     glm::vec3(far_width / 2, far_height / 2, -far_z)};

        // TRS matrices from camera
        glm::vec3 t = camera.transform.get_translation();
        glm::vec3 s = camera.transform.get_scale();
        glm::vec3 r = camera.transform.get_rotation();
        glm::vec3 r_rad = r * glm::two_pi<float>();

        glm::mat4 T = glm::translate(glm::mat4(1.0f), t);
        glm::mat4 Rx = glm::rotate(glm::mat4(1.0f), r_rad.x, glm::vec3(1, 0, 0));
        // TODO: once again I have no idea why we keep having to fix this
        glm::mat4 Ry = glm::rotate(glm::mat4(1.0f), -r_rad.y - glm::two_pi<float>() / 4, glm::vec3(0, 1, 0));
        glm::mat4 Rz = glm::rotate(glm::mat4(1.0f), r_rad.z, glm::vec3(0, 0, 1));
        glm::mat4 R = Ry * Rx * Rz;
        glm::mat4 S = glm::scale(glm::mat4(1.0f), s);
        glm::mat4 world_matrix = T * R * S;

        std::array<glm::vec3, 8> world_corners;
        int i = 0;
        for (auto &c : cam_corners) {
            glm::vec4 w = world_matrix * glm::vec4(c, 1.0f);
            world_corners[i] = glm::vec3(w);
            i++;
        }

        // Plane helper
        // NOTE: a plane is specified by the equation (n . x) = 0  which can represent any plane going through the
        // origin, if we want to allow the plane to instead pass through some arbitrary point p, then the equation
        // becomes (n . (p - x)) = 0 and now p is guarenteed to be on the plane, so we're offseting the plane by p
        // the equation can then become n.p - n.x = 0 which is equivalent to n.x + d = 0 fs d in R, so the most compact
        // way to express a plane is by n, d which is 4d vector
        auto make_plane_inward = [](const glm::vec3 &a, const glm::vec3 &b, const glm::vec3 &c) {
            glm::vec3 normal = glm::normalize(glm::cross(b - a, c - a));
            float d = -glm::dot(normal, a);
            return glm::vec4(normal, d);
        };

        glm::vec3 nbl = world_corners[0], nbr = world_corners[1], ntl = world_corners[2], ntr = world_corners[3];
        glm::vec3 fbl = world_corners[4], fbr = world_corners[5], ftl = world_corners[6], ftr = world_corners[7];

        /*
         *     ftl*--------------------------*ftr
         *        | .                      . |
         *        |    .                .    |
         *        |       .          .       |
                  |      ntl*------*ntr      |
         *        |         |      |         |
         *        |         |      |         |
                  |      nbl*------*nbr      |
         *        |       .          .       |
         *        |    .                .    |
         *        | .                      . |
         *     fbl*--------------------------*fbr
         *
         */

        /* RIGHT HAND RULE FOR CROSS PRODUCT
         *
         *                 (A x B)
         *
         *                    |
         *                    |
         *                    |
         *                    |
         *                    |
         *                    o----------   B
         *                   /
         *                  /
         *                 /
         *                /
         *
         *             A
         *
         */

        // NOTE: we use the above rule to make sure that normals are facing inside of the frustum
        planes[0] = make_plane_inward(nbl, fbl, ftl); // left
        planes[1] = make_plane_inward(nbr, ntr, ftr); // right
        planes[2] = make_plane_inward(nbl, nbr, fbr); // bottom
        planes[3] = make_plane_inward(ntl, ftl, ftr); // top
        planes[4] = make_plane_inward(ntl, ntr, nbr); // near
        planes[5] = make_plane_inward(ftr, ftl, fbl); // far

        // for (size_t i = 0; i < planes.size(); ++i) {
        //     std::cout << "Plane " << i << " = (" << planes[i].x << ", " << planes[i].y << ", " << planes[i].z << ", "
        //               << planes[i].w << ")\n";
        // }
    }

    template <draw_info::IVPLike IVPX> bool is_visible(IVPX &ivp_x) {
        return is_visible(ivp_x.xyz_positions, ivp_x.transform);
    }

    bool is_visible(const std::vector<glm::vec3> &xyz_positions, Transform &transform) override {

        update_frustum_planes();

        auto local_aabb = vertex_geometry::AxisAlignedBoundingBox(xyz_positions);
        auto corners = get_aabb_corners_world(local_aabb, transform);

        constexpr float epsilon = 1e-6f;

        // std::cout << "=== Checking object visibility ===\n";
        // std::cout << "Object AABB corners in world space:\n";
        for (size_t i = 0; i < corners.size(); ++i) {
            const auto &c = corners[i];
            // std::cout << "  Corner " << i << ": (" << c.x << ", " << c.y << ", " << c.z << ")\n";
        }

        const char *plane_names[6] = {"Left", "Right", "Bottom", "Top", "Near", "Far"};

        // Iterate over frustum planes
        for (size_t pi = 0; pi < planes.size(); ++pi) {
            const auto &plane = planes[pi];
            glm::vec3 normal(plane.x, plane.y, plane.z);

            // std::cout << "\n--- Testing against plane " << pi << " (" << plane_names[pi] << ") ---\n";
            // std::cout << "Plane normal: (" << normal.x << ", " << normal.y << ", " << normal.z << "), w = " <<
            // plane.w
            //           << "\n";

            bool all_outside = true;

            for (size_t ci = 0; ci < corners.size(); ++ci) {
                const auto &corner = corners[ci];
                float dist = glm::dot(normal, corner) + plane.w;

                bool inside = dist >= -epsilon;
                all_outside &= !inside;

                // std::cout << "  Corner " << ci << " at (" << corner.x << ", " << corner.y << ", " << corner.z << ")
                // "; std::cout << "distance to plane: " << dist << " -> " << (inside ? "INSIDE" : "OUTSIDE") << "\n";
            }

            if (all_outside) {
                // std::cout << "  All corners are outside the " << plane_names[pi] << " plane -> object CULLED\n";
                return false;
            } else {
                // std::cout << "  At least one corner is inside the " << plane_names[pi]
                //           << " plane -> continue checking\n";
            }
        }

        // std::cout << "=== Object is VISIBLE ===\n";
        return true;
    }

    // NOTE: if we ever need to redo this then I want to redo it by using connect n-gon from vertex geom.
    draw_info::IndexedVertexPositions generate_frustum_ivp(bool center_at_origin = false) {
        // Camera params
        float aspect = static_cast<float>(screen_width_px) / screen_height_px;
        float fov_y = glm::radians(camera.fov.get()); // vertical FOV
        float near_z = camera.near_plane;
        float far_z = camera.far_plane;
        far_z = 2;

        // Half-sizes of near/far planes
        float tan_half_fov_y = tanf(fov_y * 0.5f);
        float near_height = 2.0f * near_z * tan_half_fov_y;
        float near_width = near_height * aspect;
        float far_height = 2.0f * far_z * tan_half_fov_y;
        float far_width = far_height * aspect;

        // Define frustum corners in camera space
        std::array<glm::vec3, 8> cam_corners = {
            // Near plane
            glm::vec3(-near_width / 2, -near_height / 2, -near_z), glm::vec3(near_width / 2, -near_height / 2, -near_z),
            glm::vec3(-near_width / 2, near_height / 2, -near_z), glm::vec3(near_width / 2, near_height / 2, -near_z),
            // Far plane
            glm::vec3(-far_width / 2, -far_height / 2, -far_z), glm::vec3(far_width / 2, -far_height / 2, -far_z),
            glm::vec3(-far_width / 2, far_height / 2, -far_z), glm::vec3(far_width / 2, far_height / 2, -far_z)};

        std::vector<glm::vec3> world_corners;
        world_corners.reserve(8);

        // --- Manually construct TRS matrix from Transform ---
        glm::vec3 t = camera.transform.get_translation();
        glm::vec3 s = camera.transform.get_scale();
        glm::vec3 r = camera.transform.get_rotation(); // in turns (0..1)
        glm::vec3 r_rad = r * glm::two_pi<float>();    // convert turns to radians

        // Translation matrix
        glm::mat4 T = glm::translate(glm::mat4(1.0f), t);

        // Rotation matrices (Euler order: pitch=X, yaw=Y, roll=Z)
        glm::mat4 Rx = glm::rotate(glm::mat4(1.0f), r_rad.x, glm::vec3(1, 0, 0));
        glm::mat4 Ry = glm::rotate(glm::mat4(1.0f), -r_rad.y - glm::two_pi<float>() / 4, glm::vec3(0, 1, 0));
        glm::mat4 Rz = glm::rotate(glm::mat4(1.0f), r_rad.z, glm::vec3(0, 0, 1));
        glm::mat4 R = Ry * Rx * Rz; // assuming YXZ order (common for FPS camera)

        // Scale matrix
        glm::mat4 S = glm::scale(glm::mat4(1.0f), s);

        // Combine to world matrix
        glm::mat4 world_matrix = T * R * S; // TRS order

        for (auto &c : cam_corners) {
            glm::vec4 world = world_matrix * glm::vec4(c, 1.0f);
            world_corners.push_back(glm::vec3(world));
        }

        if (center_at_origin) {
            for (auto &p : world_corners) {
                p -= t; // shift apex to origin
            }
        }

        // Triangle indices
        std::vector<unsigned int> indices = {
            0, 1, 2, 1, 3, 2, // near
            4, 6, 5, 5, 6, 7, // far
            0, 2, 4, 2, 6, 4, // left
            1, 5, 3, 3, 5, 7, // right
            2, 3, 6, 3, 7, 6, // top
            0, 4, 1, 1, 4, 5  // bottom
        };

        return draw_info::IndexedVertexPositions(indices, world_corners);
    }

  private:
    FPSCamera &camera;
    const unsigned int &screen_width_px, &screen_height_px;
    std::array<glm::vec4, 6> planes; // frustum planes

    std::array<glm::vec3, 8> get_aabb_corners_world(const vertex_geometry::AxisAlignedBoundingBox &box,
                                                    Transform &transform) const {
        glm::mat4 model = transform.get_transform_matrix();
        std::array<glm::vec3, 8> corners = box.get_corners();
        for (auto &c : corners) {
            glm::vec4 world = model * glm::vec4(c, 1.0f);
            c = glm::vec3(world);
        }
        return corners;
    }
};

#endif // CULLING_HPP
