//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include <thread>
#include "Scene.hpp"
#include "Renderer.hpp"
#define TIME_TO_SLEEP 1000

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;
std::atomic_int progress = 0;
std::atomic_int finish_count = 0;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene) {
	std::vector<Vector3f> framebuffer(scene.width * scene.height);

	float scale = tan(deg2rad(scene.fov * 0.5));
	float imageAspectRatio = scene.width / (float)scene.height;
	Vector3f eye_pos(278, 273, -800);
	int m = 0;

	// change the spp value to change sample ammount
	int spp = 1024;
	int sqrt_spp = (int)(ceil(sqrt(spp)));
	spp = sqrt_spp * sqrt_spp;
	std::cout << "SPP: " << spp << "\n";
	// 14�ˣ�14*2+2=30
	const int thred = 30;
	int per = scene.height / thred;
	std::thread th[thred];
	auto renderRow = [&](uint32_t lrow, uint32_t hrow)
	{
		float step = 1.0 / sqrt_spp;
		for (uint32_t j = lrow; j < hrow; ++j) {
			for (uint32_t i = 0; i < scene.width; ++i) {
				// MSAA
				for (int k = 0; k < spp; k++) {
					// generate primary ray direction
					float cent_x = i + (k % sqrt_spp) * step + step / 2.0;
					float cent_y = j + (k / sqrt_spp) * step + step / 2.0;
					float x = (2 * cent_x / (float)scene.width - 1) *
						imageAspectRatio * scale;
					float y = (1 - 2 * (cent_y) / (float)scene.height) * scale;

					Vector3f dir = normalize(Vector3f(-x, y, 1));
					framebuffer[(int)(j * scene.width + i)] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
				}
			}
			progress++;
		}
		finish_count++;
	};
	std::thread prog;
	auto updateProgress = [&]()
	{
		while (finish_count < thred) {
			UpdateProgress(progress / (float)scene.height);
			std::this_thread::sleep_for(std::chrono::milliseconds(TIME_TO_SLEEP));
		}
	};
	for (int i = 0; i < thred; i++)
		th[i] = std::thread(renderRow, i * per, (i + 1) * per);
	prog = std::thread(updateProgress);
	for (int i = 0; i < thred; i++)
		th[i].join();
	prog.join();

	UpdateProgress(1.f);

	// save framebuffer to file
	FILE* fp = fopen("binary.ppm", "wb");
	(void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
	for (auto i = 0; i < scene.height * scene.width; ++i) {
		static unsigned char color[3];
		color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
		color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
		color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
		fwrite(color, 1, 3, fp);
	}
	fclose(fp);
}
