#include "pixy_uorb_main.h"
#include <cmath>
#include <math.h>


float min_vect_procent_1 = 0.3f;
float min_vect_procent_2 = 0.85f;

using namespace matrix;

static int daemon_task; /* Handle of deamon task / thread */

bool threadShouldExit_uorb = false;
bool threadIsRunning_uorb = false;

float v0[10], v1[10];
int i0 = 0, i1 = 0;

//calculate the distance between the simple vector and the vertical line that splits the frame in two (39 is the middle of the frame)
float dist(Vec vect)
{
	float x0 = (float)vect.m_x0;
	float x1 = (float)vect.m_x1;
	float y0 = (float)vect.m_y0;
	float y1 = (float)vect.m_y1;

	float vertical_x = 39.0f;
	float epsilon = 0.000001f; // Small threshold value

	// Check if the line segment is vertical and overlapping the fixed vertical line
	if (std::abs(x0 - x1) < epsilon && std::abs(x0 - vertical_x) < epsilon) {
		return 0.0f;
	}

	// Calculate the intersection point of the line defined by the line segment and the vertical line
	float slope = (y1 - y0) / (x1 - x0);
	float intercept = y0 - slope * x0;
	float intersection_y = slope * vertical_x + intercept;

	// Check if the intersection point lies within the line segment's y bounds
	if ((intersection_y >= y0 && intersection_y <= y1) || (intersection_y >= y1 && intersection_y <= y0)) {
		float mid_x = (x0 + x1) / 2.0f;
		float mid_y = (y0 + y1) / 2.0f;
		//dist is
		float dist = sqrt(pow((float)(mid_x - 39.0f), 2) + pow((float)(mid_y - 26), 2));
		//return 0.0f;
		return dist;
	}

	// Calculate the distances from the ends of the line segment to the vertical line
	float dist_start = std::abs(x0 - vertical_x);
	float dist_end = std::abs(x1 - vertical_x);

	// Return the minimum distance between the line segment and the vertical line
	return dist_start < dist_end ? dist_start : dist_end;

}

bool pointsAreClose(const int x0, const int y0, const int x1, const int y1, double threshold)
{
	double dist = std::sqrt(std::pow((double)(x1 - x0), 2) + std::pow((double)(y1 - y0), 2));
	return dist <= threshold;
}

bool linesIntersectAtEndpoint(Vec line0, Vec line1, double threshold)
{
	return pointsAreClose(line0.m_x0, line0.m_y0, line1.m_x0, line1.m_y0, threshold)
	       || pointsAreClose(line0.m_x0, line0.m_y0, line1.m_x1, line1.m_y1, threshold) ||
	       pointsAreClose(line0.m_x1, line0.m_y1, line1.m_x0, line1.m_y0, threshold)
	       || pointsAreClose(line0.m_x1, line0.m_y1, line1.m_x1, line1.m_y1, threshold);
}

void printIntersections(Intersection_t intersections[10], int numIntersections)
{
	for (int i = 0; i < numIntersections; i++) {
		Vec vec1;
		Vec vec0;
		vec0.m_x0 = intersections[i].vec0_x0;
		vec0.m_x1 = intersections[i].vec0_x1;
		vec0.m_y0 = intersections[i].vec0_y0;
		vec0.m_y1 = intersections[i].vec0_y1;
		vec0.m_index = intersections[i].vec0_index;

		vec1.m_x0 = intersections[i].vec1_x0;
		vec1.m_x1 = intersections[i].vec1_x1;
		vec1.m_y0 = intersections[i].vec1_y0;
		vec1.m_y1 = intersections[i].vec1_y1;
		vec1.m_index = intersections[i].vec1_index;

		printf("Intersection %d: Line1: (%d, %d) (%d, %d) Index: %d ; Line2: (%d, %d) (%d, %d) Index: %d\n", i,
		       vec0.m_x0, vec0.m_y0, vec0.m_x1, vec0.m_y1, vec0.m_index,
		       vec1.m_x0, vec1.m_y0, vec1.m_x1, vec1.m_y1, vec1.m_index);
	}
}

int getIntersections(Pixy2 &pixy, Intersection_t intersections[10])
{
	int numIntersections = 0;
	double threshold = 6.0;

	for (int i = 0; i < pixy.line.numVectors - 1; i++) {
		for (int j = i + 1; j < pixy.line.numVectors; j++) {
			Vec line0 = pixy.line.vectors[i];
			Vec line1 = pixy.line.vectors[j];
			//intersections[numIntersections].vec0_x0 = 2123;

			if (linesIntersectAtEndpoint(line0, line1, threshold)) {
				intersections[numIntersections].vec0_x0 = line0.m_x0;
				intersections[numIntersections].vec0_x1 = line0.m_x1;
				intersections[numIntersections].vec0_y0 = line0.m_y0;
				intersections[numIntersections].vec0_y1 = line0.m_y1;
				intersections[numIntersections].vec0_index = line0.m_index;

				intersections[numIntersections].vec1_x0 = line1.m_x0;
				intersections[numIntersections].vec1_x1 = line1.m_x1;
				intersections[numIntersections].vec1_y0 = line1.m_y0;
				intersections[numIntersections].vec1_y1 = line1.m_y1;
				intersections[numIntersections].vec1_index = line1.m_index;
				//printf("Inters %d\n", numIntersections);
				numIntersections++;

			}
		}
	}

	return numIntersections;
}
void printVect(Vec line, const char name[])
{
	printf("%s: (%d, %d) (%d, %d) Index: %d\n", name, line.m_x0, line.m_y0, line.m_x1, line.m_y1, line.m_index);
}
void vec_reset(float vec[])
{
	for (int i = 0; i < 10; i++) {
		vec[i] = 0;
	}
}

float medie(float v[], float i_aux)
{
	//printf("Hello");
	float medie = 0;

	for (int i = 0; i < i_aux; i++) {
		medie += v[i];
	}

	return (double)(medie / i_aux);
}

float procent(float v[], float panta,  float i)
{
	if ((double)(medie(v, i)) > (double)(panta)) {
		return (double)(panta / medie(v, i));
	}

	return (double)(medie(v, i) / panta);
}
float procent_val(float panta_cur, float panta_prev)
{
	if ((double)(panta_cur) > (double)(panta_prev)) {
		return (double)(panta_prev / panta_cur);
	}

	return (double)(panta_cur / panta_prev);
}

float lineLength(Vec line)
{
	float x0 = (float)line.m_x0;
	float x1 = (float)line.m_x1;
	float y0 = (float)line.m_y0;
	float y1 = (float)line.m_y1;
	float length = sqrt(pow((float)(x1 - x0), 2) + pow((float)(y1 - y0), 2));

	return (double)(length);
}

void assignVect(Vec &vec1, Vec vec2)
{
	vec1.m_x0 = vec2.m_x0;
	vec1.m_x1 = vec2.m_x1;
	vec1.m_y0 = vec2.m_y0;
	vec1.m_y1 = vec2.m_y1;
	vec1.m_index = vec2.m_index;
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

		return (double)(panta);
	}
}
float slope(int x0, int y0, int x1, int y1)
{
	if (x0 - x1 == 0) {
		return 9999.0f;

	} else {
		float panta = (y1 - y0) / (x1 - x0);
		return panta;
	}
}
// float norm(int x0, int y0, int x1, int y1)
// {
// 	float norm = sqrt(pow((float)(x1 - x0), 2) + pow((float)(y1 - y0), 2));
// 	return (double)(norm);
// }
double angle_between_segments(int line1_x0, int line1_y0, int line1_x1, int line1_y1,
			      int line2_x0, int line2_y0, int line2_x1, int line2_y1)
{
	int vec_a_x = line1_x1 - line1_x0;
	int vec_a_y = line1_y1 - line1_y0;
	int vec_b_x = line2_x1 - line2_x0;
	int vec_b_y = line2_y1 - line2_y0;

	double norm_a = std::sqrt(vec_a_x * vec_a_x + vec_a_y * vec_a_y);
	double norm_b = std::sqrt(vec_b_x * vec_b_x + vec_b_y * vec_b_y);

	//printf("%f %f\n", (double)norm_a, (double)norm_b);

	if (std::abs(norm_a) < 0.01f || std::abs(norm_b) < 0.01f) {
		// One or both of the segments have zero length, so the angle is undefined.
		//printf("undefined\n");
		return 0;
	}

	double dot = vec_a_x * vec_b_x + vec_a_y * vec_b_y;
	double cos_angle = dot / (norm_a * norm_b);

	// Ensure cos_angle is within the valid range for acos function [-1, 1]
	if (cos_angle > 1.0) {
		cos_angle = 1.0;

	} else if (cos_angle < -1.0) {
		cos_angle = -1.0;
	}

	double angle_rad = std::acos(cos_angle);

	// Convert radians to degrees
	double angle_deg = angle_rad * 180.0 / M_PI;

	return angle_deg;
}
// get angle in degrees between two lines
// float angleLines(int line1_x0, int line1_x1, int line1_y0,
// 		 int line1_y1, int line2_x0, int line2_x1,
// 		 int line2_y0, int line2_y1)
// {
// 	float slope1 = slope(line1_x0, line1_y0, line1_x1, line1_y1);
// 	float slope2 = slope(line2_x0, line2_y0, line2_x1, line2_y1);

// 	if (fabs(1 + slope1 * slope2) > 0.01) {

// 	}

// 	float angle = std::atan((double)((slope1 - slope2) / (1 + slope1 * slope2))) * 180 / M_PI;

// 	return angle;
// }
uint8_t get_nums_vectors(Vec &vec1, Vec &vec2)
{

	uint8_t numVectors = 0;

	if (!(vec1.m_x0 == 0 && vec1.m_x1 == 0 && vec1.m_y0 == 0 && vec1.m_y1 == 0)) {
		numVectors++;
	}

	if (!(vec2.m_x0 == 0 && vec2.m_x1 == 0 && vec2.m_y0 == 0 && vec2.m_y1 == 0)) {
		numVectors++;
	}

	return numVectors;
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
		float absSlope = std::fabs((double)(slope(line)));

		if (absSlope < 0.5f) {
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
		if (std::abs(height1 - height2) < 15) {
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

	if (argc >= 2) {
		min_vect_procent_1 = atof(argv[1]);
		min_vect_procent_2 = atof(argv[2]);
	}

	if (argc >= 2) {
		printf("argv[%d] = %s\n", 1, argv[1]);
		printf("argv[%d] = %s\n", 2, argv[2]);

	}

	//printf("min_vect_procent_1 = %f, min_vect_procent_2 = %f\n", (double)min_vect_procent_1, (double)min_vect_procent_2);
	struct pixy_vector_s _pixy_vector;

	//struct start_line_detected_s _start_line_detected;

	int dr = 0, st = 0;

	/* Pixy2 Instance */
	Pixy2 pixy;

	bool wait = 1; // needed fOBSTACLEor waiting for valid data

	usleep(5000);  // give pixy time to init

	static uint8_t index0 = 0;

	static uint8_t index1 = 0;

	//bool used to detect when the mostVerticalVect line transitions from a valid line to 0,0,0,0 line
	//bool exitingIntersection = false;
	//bool isValid = false;

	// Make sure pixy is ready
	if (pixy.init() == 0) {

		// Print Pixy details to confirm Pixy is publishing data over i2c
		pixy.getVersion();
		pixy.version->print();
		usleep(1000);
		Vec vect0;
		Vec vect1;
		Vec vect0_old;
		Vec vect1_old;
		//Vec intersection_vect = {0};
		Vec mostVerticalVect = {0};
		Intersection_t intersections[10] = {0};
		Vec badIntersectionVec = {0};
		int numIntersections = 0;

		// Loop indefinitely and publish vector data
		while (1) {
			init_vectors(vect0, vect1);
			st = 0;
			dr = 0;
			//int nr_of_consecutive_start_lines = 0;s
			pixy.line.getAllFeatures(LINE_VECTOR, wait); // get line vectors from pixy

			numIntersections = getIntersections(pixy, intersections);

			//printIntersections(intersections, numIntersections);
			//numIntersections++;
			if (numIntersections > 0) {
				float slope0 = slope(intersections[0].vec0_x0, intersections[0].vec0_y0, intersections[0].vec0_x1,
						     intersections[0].vec0_y1);
				float slope1 = slope(intersections[0].vec1_x0, intersections[0].vec1_y0, intersections[0].vec1_x1,
						     intersections[0].vec1_y1);
				Vec vecInters0 = {(u_int8_t)intersections[0].vec0_x0, (u_int8_t)intersections[0].vec0_y0,
						  (u_int8_t)intersections[0].vec0_x1, (u_int8_t)intersections[0].vec0_y1,
						  (u_int8_t)intersections[0].vec0_index, 0
						 };
				Vec vecInters1 = {
					(u_int8_t)intersections[0].vec1_x0, (u_int8_t)intersections[0].vec1_y0,
					(u_int8_t)intersections[0].vec1_x1, (u_int8_t)intersections[0].vec1_y1,
					(u_int8_t)intersections[0].vec1_index, 0
				};
				double angle = angle_between_segments(
						       intersections[0].vec0_x0, intersections[0].vec0_y0,
						       intersections[0].vec0_x1, intersections[0].vec0_y1,
						       intersections[0].vec1_x0, intersections[0].vec1_y0,
						       intersections[0].vec1_x1, intersections[0].vec1_y1);

				if (abs(angle) > 80.0 && abs(angle) < 100.0) {
					// printf("Angle %f\n", angle);
					// printf("Line1: (%2d, %2d) (%2d, %2d) Index: %3d ; Line2: (%2d, %2d) (%2d, %2d) Index: %3d\n",
					//        intersections[0].vec0_x0, intersections[0].vec0_y0, intersections[0].vec0_x1, intersections[0].vec0_y1,
					//        intersections[0].vec0_index,
					//        intersections[0].vec1_x0, intersections[0].vec1_y0, intersections[0].vec1_x1, intersections[0].vec1_y1,
					//        intersections[0].vec1_index);
					//printf("slope0 %lf,slope1 %lf\n", (double)slope0, (double)slope1);

					if (fabs(slope0) > fabs(slope1)) {
						assignVect(mostVerticalVect, vecInters0);
						assignVect(badIntersectionVec, vecInters1);

					} else {
						assignVect(mostVerticalVect, vecInters1);
						assignVect(badIntersectionVec, vecInters0);
					}

					printVect(mostVerticalVect, "Most vertical vector");
					printVect(badIntersectionVec, "Bad intersection vector");
					//isValid = true;
				}

			} else {
				// if (isValid == true) {
				// 	isValid = false;
				// 	//exitingIntersection = true;
				// 	//printf("Iese\n");
				// }
			}

			//printVect(mostVerticalVect, "Most vertical vector");
			float length0_min = 9999.0f;
			float length1_min = 9999.0f;

			if (pixy.line.numVectors) {
				// extrag cei mai lungi vectori verticali
				for (int i = 0; i < pixy.line.numVectors; i++) {
					Vec line = pixy.line.vectors[i];
					line.m_y0 = 51 - line.m_y0;
					line.m_y1 = 51 - line.m_y1;
					//float length = lineLength(line);
					float absSlope = std::fabs((double)(slope(
							line)));// now we are comparing the true floating point number of the slope

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

					// if vector is vertical and at the left of the frame
					if (/*(double)(length) > (double)(lineLength(vect0))*/dist(line) < length0_min && (double)(absSlope) >= 0.3
							&& (double)(line.m_x0) < 39) {
						// vector is good
						vect0 = line;
						st = 1;
						length0_min = dist(line);
					}

					// if vector is vertical and at the right of the frame
					if (/*(double)(length) > (double)(lineLength(vect1))*/dist(line) < length1_min && (double)(absSlope) >= 0.3
							&& (double)(line.m_x0) >= 39) {
						// vector is good
						vect1 = line;
						dr = 1;
						length1_min = dist(line);
					}
				}

				//if (numIntersections != 0 && exitingIntersection == true) {
				//exitingIntersection = false;

				// if (st = 0) {
				// 	if()
				// 	assignVect(vect1, mostVerticalVect);

				// } else if (dr = 0) {
				// 	assignVect(vect0, mostVerticalVect);
				// }
				//printVect(mostVerticalVect, "Most vertical: ");

				if (vect1.m_index == badIntersectionVec.m_index) {
					printVect(vect1, "Vector prost1: ");
					//printVect(mostVerticalVect, "Vector bun1: ");
					assignVect(vect1, mostVerticalVect);
					printVect(vect1, "Vector bun1: ");
				}

				if (vect0.m_index == badIntersectionVec.m_index) {
					printVect(vect0, "Vector prost0: ");
					//printVect(mostVerticalVect, "Vector bun0: ");
					assignVect(vect0, mostVerticalVect);
					printVect(vect0, "Vector bun0: ");
				}

				printVect(vect0, "Vector 0: ");
				//}

				if (index0 != vect0.m_index && st == 1
				    && procent_val(std::fabs((double)(slope(vect0))),
						   std::fabs((double)(slope(vect0_old)))) > min_vect_procent_1) {
					vect0_old = vect0;
					index0 = vect0.m_index;
//					vec_reset(v0);
					i0 = 0;


				} else if (st == 1
					   && procent_val(std::fabs((double)(slope(vect0))),
							  std::fabs((double)(slope(vect0_old)))) > min_vect_procent_2) {
					vect0 = vect0_old;
					index0 = 255;
					//	printf("vect old\n");

				} else {
					//	printf("vect normal\n");
				}

				if (index1 != vect1.m_index  && dr == 1
				    && procent_val(std::fabs((double)(slope(vect1))),
						   std::fabs((double)(slope(vect1_old)))) > min_vect_procent_1) {
					vect1_old = vect1;
					index1 = vect1.m_index;
					//vec_reset(v1);
					i1 = 0;

				} else if (dr == 1
					   && procent_val(std::fabs((double)(slope(vect1))),
							  std::fabs((double)(slope(vect1_old)))) > min_vect_procent_2) {
					vect1 = vect1_old;
					index1 = 255;
					//	printf("vect old\n");

				} else {
					//	printf("vect normal\n");

				}

				printVect(vect0, "Vector 0 dupa indecsi: ");
				// printf("vect0: x0= %d, y0=%d, x1=%d, y1=%d , m=%lf , index=%d\n", vect0.m_x0, vect0.m_y0, vect0.m_x1, vect0.m_y1,
				//        std::fabs((double)(slope(vect0))), vect0.m_index);
				// printf("vect1: x0= %d, y0=%d, x1=%d, y1=%d, m=%lf, index=%d\n", vect1.m_x0, vect1.m_y0, vect1.m_x1, vect1.m_y1,
				//        std::fabs((double)(slope(vect1))), vect1.m_index);
				// printf("\n");

				// code that counts the number of consecutive start lines using the start line detection function
				// if (detectStartLine(pixy)) {
				// 	nr_of_consecutive_start_lines++;

				// } else {
				// 	nr_of_consecutive_start_lines = 0;
				// }

				// // if there are more than 5 consecutive start lines, then publish start line detected
				// if (nr_of_consecutive_start_lines > 0) {
				// 	// publish start line detected
				// 	_start_line_detected.start_line_detected = true;
				// 	_start_line_detected.timestamp = hrt_absolute_time();
				// 	_start_line_detected_pub.publish(_start_line_detected);

				// } else {
				// 	_start_line_detected.start_line_detected = false;
				// 	_start_line_detected.timestamp = hrt_absolute_time();
				// 	_start_line_detected_pub.publish(_start_line_detected);
				// }

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
				}

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

