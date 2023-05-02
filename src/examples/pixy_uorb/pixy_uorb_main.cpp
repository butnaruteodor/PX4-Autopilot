#include "pixy_uorb_main.h"
#include <cmath>


void clear_console()
{
#ifdef _WIN32
	std::system("CLS");
#elif __unix__
	std::system("clear");
#else
	// Unsupported platform
#endif
}

using namespace matrix;

static int daemon_task; /* Handle of deamon task / thread */

bool threadShouldExit_uorb = false;
bool threadIsRunning_uorb = false;

//float v0[10], v1[10];
//int i0 = 0, i1 = 0;

// float medie(float v[], float i_aux)
// {
//     //printf("Hello");
//     float medie = 0;

//     for (int i = 0; i < i_aux; i++) {
//         medie += v[i];
//     }

//     return static_cast<double>(medie / i_aux);
// }

// float procent(float v[], float panta,  float i)
// {
//     if (static_cast<double>(medie(v, i)) > static_cast<double>(panta)) {
//         return static_cast<double>(panta / medie(v, i));
//     }

//     return static_cast<double>(medie(v, i) / panta);
// }

//number of values diffrent from 0 in a vector
int num_values(int v[])
{
	int num = 0;

	for (int i = 0; i < 10; i++) {
		if (v[i] != 0) {
			num++;
		}
	}

	return num;
}
//reset array to 0
void reset_array(int v[])
{
	for (int i = 0; i < 10; i++) {
		v[i] = 0;
	}
}
//distance between two different end points
float distance_endpoints(Vec line1, Vec line2)
{
	float x0 = (float)line1.m_x0;
	float x1 = (float)line2.m_x0;
	float y0 = (float)line1.m_y0;
	float y1 = (float)line2.m_y0;
	float length = sqrt(pow((float)(x1 - x0), 2) + pow((float)(y1 - y0), 2));

	return static_cast<double>(length);
}
//if the distance between the end points (x0,y0 from line1 and x1,y1 from line2) is less than 7, then the lines are intersecting
bool intersection(Vec line1, Vec line2)
{
	if (distance_endpoints(line1, line2) < 7) {
		return true;
	}

	return false;
}

float slope(Vec lines)
{
	if (lines.m_x0 - lines.m_x1 == 0) {
		return 9999.0f;

	} else {
		float x0 = (float)lines.m_x0;
		float x1 = (float)lines.m_x1;
		float y0 = (float)lines.m_y0;
		float y1 = (float)lines.m_y1;
		float panta = (y1 - y0) / (x1 - x0);

		return static_cast<double>(panta);
	}
}

//we have a vect_left/vect_right that keeps track of all the left/right lines and we want to see if any of them returns true for intersection func with themselves
Vec intersecting_good_line(Vec vect[])
{
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++)
			if (intersection(vect[i], vect[j])) {
				//if slope of one vector represents a more vertical line than the other, then we return the one with the more vertical line
				if (std::fabs(slope(vect[i])) > std::fabs(slope(vect[j])) && slope(vect[i]) <= 9998.0f) {
					return vect[i];

				} else {
					return vect[j];
				}


			}
	}

	printf("not intersecting\n");
	return {0};
}

float lineLength(Vec line)
{
	float x0 = (float)line.m_x0;
	float x1 = (float)line.m_x1;
	float y0 = (float)line.m_y0;
	float y1 = (float)line.m_y1;
	float length = sqrt(pow((float)(x1 - x0), 2) + pow((float)(y1 - y0), 2));

	return static_cast<double>(length);
}

void init_vectors(Vec &vect1, Vec &vect2)
{
	vect1.m_x0 = 0;
	vect1.m_x1 = 0;
	vect1.m_y0 = 0;
	vect1.m_y1 = 0;

	vect2.m_x0 = 0;
	vect2.m_x1 = 0;
	vect2.m_y0 = 0;
	vect2.m_y1 = 0;
}

bool detectStartLine(Pixy2 &pixy)
{
	// if there are more than 6 vectors, then there surely something is wrong, there is no start line
	if (pixy.line.numVectors >= 6) {
		return false;
	}

	// buffer for all horizontal vectors
	Vec horizontalVectors[pixy.line.numVectors];
	int numHorizontalVectors = 0;

	// extract horizontal vectors
	for (int i = 0; i < pixy.line.numVectors; i++) {
		Vec line = pixy.line.vectors[i];
		float absSlope = std::fabs(static_cast<double>(slope(line)));

		if (absSlope < 0.2f) {
			horizontalVectors[numHorizontalVectors].m_x0 = line.m_x0;
			horizontalVectors[numHorizontalVectors].m_x1 = line.m_x1;
			horizontalVectors[numHorizontalVectors].m_y0 = line.m_y0;
			horizontalVectors[numHorizontalVectors].m_y1 = line.m_y1;
			numHorizontalVectors++;
		}
	}

	// if there are more than 2 horizontal vectors there could be a start line
	if (numHorizontalVectors >= 2) {
		int height1, height2;
		height1 = (horizontalVectors[0].m_y1 + horizontalVectors[0].m_y0) / 2;
		height2 = (horizontalVectors[1].m_y1 + horizontalVectors[1].m_y0) / 2;

		// if the height difference is less than 5 pixels, then there is a start line
		if (std::abs(height1 - height2) < 5) {
			return true;
		}
	}

	return false;
}

int pixy_uorb_thread_main(int argc, char **argv)
{
	threadIsRunning_uorb = true;

	/* Publication of uORB messages */
	uORB::Publication<pixy_vector_s> _pixy_vector_pub{ORB_ID(pixy_vector)};
	uORB::Publication<start_line_detected_s> _start_line_detected_pub{ORB_ID(start_line_detected)};
	uORB::SubscriptionData<distance_sensor_s> distance_sub{ORB_ID(distance_sensor)};

	struct pixy_vector_s _pixy_vector;
	struct start_line_detected_s _start_line_detected;
	int dr = 0, st = 0;

	/* Pixy2 Instance */
	Pixy2 pixy;
	bool wait = 1; // needed fOBSTACLEor waiting for valid data
	usleep(5000);  // give pixy time to init

	// Make sure pixy is ready
	if (pixy.init() == 0) {

		// Print Pixy details to confirm Pixy is publishing data over i2c
		pixy.getVersion();
		pixy.version->print();

		int v_index_left[10];
		int v_index_right[10];
		usleep(1000);
		Vec vect0;
		Vec vect1;

		// Loop indefinitely and publish vector data
		while (1) {
			reset_array(v_index_left);
			reset_array(v_index_right);
			init_vectors(vect0, vect1);
			st = 0;
			dr = 0;
			int nr_of_consecutive_start_lines = 0;
			int ok = 0;
			pixy.line.getAllFeatures(LINE_VECTOR, wait); // get line vector data

			if (pixy.line.numVectors) {
				for (int i = 0; i < pixy.line.numVectors; i++) {
					//check if there are vectors on the same half than increase the index
					if (pixy.line.vectors[i].m_x0 < 39) {
						v_index_left[i]++;

					} else if (pixy.line.vectors[i].m_x0 >= 39) {
						v_index_right[i]++;
					}
				}


				printf("left %d\n", num_values(v_index_left));
				printf("right %d\n", num_values(v_index_right));
				printf("\n");

				//check how many vectors are in each half
				if (num_values(v_index_left) > 1) {
					Vec vect_left[2];
					int nr = -1;

					for (int i = 0; i < num_values(v_index_left); i++) {
						if (v_index_left[i] > 0) { // we go through only in those vectors that have a value at an certain index
							vect_left[++nr] = pixy.line.vectors[i];
						}

						printf("left more\n");
					}

					vect0 = intersecting_good_line(vect_left);
					vect0.m_y0 = 51 - vect0.m_y0;
					vect0.m_y1 = 51 - vect0.m_y1;
					ok = 1;

				} else if (num_values(v_index_right) > 1) {
					Vec vect_right[3] = {0};
					int nr = -1;

					for (int i = 0; i < num_values(v_index_right); i++) {
						if (v_index_right[i] > 0) { // we go through only in those vectors that have a value at an certain index
							vect_right[++nr] = pixy.line.vectors[i];
						}

						printf("right more\n");
					}

					vect1 = intersecting_good_line(vect_right);
					vect1.m_y0 = 51 - vect1.m_y0;
					vect1.m_y1 = 51 - vect1.m_y1;
					ok = 1;

				} else { ok = 0; }

				printf("ok==%d\n", ok);

				if (ok == 0) { // extrag cei mai lungi vectori verticali
					for (int i = 0; i < pixy.line.numVectors; i++) {
						Vec line = pixy.line.vectors[i];
						// int number_intersections = pixy.line.numIntersections;
						line.m_y0 = 51 - line.m_y0; // because the (0,0) are is in the upper left corner
						line.m_y1 = 51 - line.m_y1;

						float length = lineLength(line);
						float absSlope = std::fabs(static_cast<double>(slope(
										   line)));// now we are comparing the true floating point number of the slope

						//printf("merge aici\n");
						// check if line is upside down
						if (line.m_y1 < line.m_y0) {
							// swap endpoints
							int aux = line.m_y0;
							line.m_y0 = line.m_y1;
							line.m_y1 = aux;

							aux = line.m_x0;
							line.m_x0 = line.m_x1;
							line.m_x1 = aux;

						} else {
							// dont swap/do nothing
						}

						// if the vector is niether half is not an intersection, program will continue from here
						// if vector is vertical and at the left of the frame
						if (static_cast<double>(length) > static_cast<double>(lineLength(vect0)) && static_cast<double>(absSlope) >= 0.3
						    && static_cast<double>(line.m_x0) < 39) {
							vect0 = line;
							st = 1;
						}

						// if vector is vertical and at the right of the frame
						if (static_cast<double>(length) > static_cast<double>(lineLength(vect1)) && static_cast<double>(absSlope) >= 0.3
						    && static_cast<double>(line.m_x0) >= 39) {
							vect1 = line;
							dr = 1;
						}
					}
				}

			}

			// printf("vect0: x0= %d, y0=%d, x1=%d, y1=%d , m=%lf , index=%d\n", vect0.m_x0, vect0.m_y0, vect0.m_x1, vect0.m_y1,
			//        std::fabs(static_cast<double>(slope(vect0))), vect0.m_index);
			// printf("vect1: x0= %d, y0=%d, x1=%d, y1=%d, m=%lf, index=%d\n", vect1.m_x0, vect1.m_y0, vect1.m_x1, vect1.m_y1,
			//        std::fabs(static_cast<double>(slope(vect1))), vect1.m_index);
			// printf("\n");

			// code that counts the number of consecutive start lines using the start line detection function
			if (detectStartLine(pixy)) {
				nr_of_consecutive_start_lines++;

			} else {
				nr_of_consecutive_start_lines = 0;
			}

			// if there are more than 5 consecutive start lines, then publish start line detected
			if (nr_of_consecutive_start_lines > 0) {
				// publish start line detected
				_start_line_detected.start_line_detected = true;
				_start_line_detected.timestamp = hrt_absolute_time();
				_start_line_detected_pub.publish(_start_line_detected);

			} else {
				_start_line_detected.start_line_detected = false;
				_start_line_detected.timestamp = hrt_absolute_time();
				_start_line_detected_pub.publish(_start_line_detected);
			}

			if (pixy.line.numVectors == 1) {
				// only one vector found
				if (st == 1) {
					_pixy_vector.m0_x0 = vect0.m_x0;
					_pixy_vector.m0_x1 = vect0.m_x1;
					_pixy_vector.m0_y0 = vect0.m_y0;
					_pixy_vector.m0_y1 = vect0.m_y1;
					_pixy_vector.m1_x0 = 0;
					_pixy_vector.m1_x1 = 0;
					_pixy_vector.m1_y0 = 0;
					_pixy_vector.m1_y1 = 0;
					// printf("st == 1\n");

				}

				if (dr == 1) {
					_pixy_vector.m1_x0 = vect1.m_x0;
					_pixy_vector.m1_x1 = vect1.m_x1;
					_pixy_vector.m1_y0 = vect1.m_y0;
					_pixy_vector.m1_y1 = vect1.m_y1;
					_pixy_vector.m0_x0 = 0;
					_pixy_vector.m0_x1 = 0;
					_pixy_vector.m0_y0 = 0;
					_pixy_vector.m0_y1 = 0;
					// printf("dr == 1\n");
				}
			}

			if (pixy.line.numVectors > 1) {

				_pixy_vector.m0_x0 = vect0.m_x0;
				_pixy_vector.m0_x1 = vect0.m_x1;
				_pixy_vector.m0_y0 = vect0.m_y0;
				_pixy_vector.m0_y1 = vect0.m_y1;
				_pixy_vector.m1_x0 = vect1.m_x0;
				_pixy_vector.m1_x1 = vect1.m_x1;
				_pixy_vector.m1_y0 = vect1.m_y0;
				_pixy_vector.m1_y1 = vect1.m_y1;

				//printf("st dr\n");

			} else {
				// no vectors returned by pixy

				_pixy_vector.m1_x0 = 0;
				_pixy_vector.m1_x1 = 0;
				_pixy_vector.m1_y0 = 0;
				_pixy_vector.m1_y1 = 0;

				_pixy_vector.m0_x0 = 0;
				_pixy_vector.m0_x1 = 0;
				_pixy_vector.m0_y0 = 0;
				_pixy_vector.m0_y1 = 0;
			}

			_pixy_vector.timestamp = hrt_absolute_time();
			_pixy_vector_pub.publish(_pixy_vector);

			if (threadShouldExit_uorb) {
				threadIsRunning_uorb = false;
				PX4_INFO("Exit Pixy uORB Thread!\n");
				return 1;
			}
		}
	}

	return 0;
}

extern "C" __EXPORT int pixy_uorb_main(int argc, char *argv[]);
int pixy_uorb_main(int argc, char *argv[])
{

	if (argc < 2) {
		PX4_WARN("usage: pixy_uorb {start|stop|status}\n");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (threadIsRunning_uorb) {
			PX4_INFO("already running\n");
			/* this is not an error */
			return 0;
		}

		threadShouldExit_uorb = false;
		daemon_task = px4_task_spawn_cmd("pixy_uorb",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 5,
						 2000,
						 pixy_uorb_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		threadShouldExit_uorb = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (threadIsRunning_uorb) {
			PX4_INFO("is running\n");

		} else {
			PX4_INFO("not started\n");
		}

		return 0;
	}

	PX4_WARN("usage: pixy_uorb {start|stop|status}\n");
	return 1;
}

