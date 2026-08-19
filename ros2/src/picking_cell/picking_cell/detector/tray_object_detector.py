import math
from pathlib import Path
import cv2
import numpy as np


class TrayObjectDetector:
    def __init__(self, warp_width=800, warp_height=520):
        self.warp_width = warp_width
        self.warp_height = warp_height
        self.anchor_x = np.array([0.130, 0.345, 0.560, 0.775], dtype=np.float32)
        self.anchor_y = np.array([0.120, 0.405, 0.690], dtype=np.float32)

        # 같은 열/행의 중심에서 너무 벗어나면 반사광 오검출로 판단
        self.max_column_deviation = self.warp_width * 0.035
        self.max_row_deviation = self.warp_height * 0.050

    def process(self, image):
        self._validate_image(image)

        tray_quad = self._find_tray_quad(image)
        warped_image, inverse_transform = self._warp_tray(image, tray_quad)
        yellow_mask, top_hat, combined_mask = self._build_marker_masks(warped_image)

        detections = self._detect_grid_objects(yellow_mask, top_hat, combined_mask)
        detections = self._regularize_grid(detections)

        for detection in detections:
            image_x, image_y = self._transform_point(
                (detection["warped_x"], detection["warped_y"]), inverse_transform
            )
            detection["image_x"] = image_x
            detection["image_y"] = image_y

        result_image = self._draw_result(image, detections, tray_quad, inverse_transform)
        return result_image, detections

    def process_file(self, input_path, output_path=None):
        input_path = Path(input_path)
        image = cv2.imread(str(input_path))

        if image is None:
            raise FileNotFoundError(f"이미지를 불러올 수 없습니다: {input_path}")

        result_image, detections = self.process(image)

        if output_path is not None:
            output_path = Path(output_path)
            output_path.parent.mkdir(parents=True, exist_ok=True)

            if not cv2.imwrite(str(output_path), result_image):
                raise RuntimeError(f"결과 이미지를 저장하지 못했습니다: {output_path}")

        return result_image, detections

    def __call__(self, image):
        return self.process(image)

    @staticmethod
    def _validate_image(image):
        if image is None:
            raise ValueError("입력 이미지가 None입니다.")
        if not isinstance(image, np.ndarray):
            raise TypeError("입력 이미지는 numpy.ndarray 형식이어야 합니다.")
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("입력 이미지는 BGR 3채널 이미지여야 합니다.")
        if image.size == 0:
            raise ValueError("입력 이미지가 비어 있습니다.")

    @staticmethod
    def _order_quad_points(points):
        points = np.asarray(points, dtype=np.float32).reshape(4, 2)
        point_sum = points.sum(axis=1)
        point_diff = np.diff(points, axis=1).reshape(-1)

        return np.array([
            points[np.argmin(point_sum)],
            points[np.argmin(point_diff)],
            points[np.argmax(point_sum)],
            points[np.argmax(point_diff)]
        ], dtype=np.float32)

    def _find_tray_quad(self, image):
        height, width = image.shape[:2]

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 0)

        dark_threshold = int(np.clip(np.percentile(gray, 18), 35, 90))
        dark_mask = cv2.inRange(gray, 0, dark_threshold)

        kernel_width = max(7, (width // 60) | 1)
        kernel_height = max(7, (height // 45) | 1)
        close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_width, kernel_height))

        dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_CLOSE, close_kernel, iterations=2)
        dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8), iterations=1)

        contours, _ = cv2.findContours(dark_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        image_area = float(width * height)
        best_rect = None
        best_score = -1.0

        for contour in contours:
            contour_area = cv2.contourArea(contour)

            if contour_area < image_area * 0.04 or contour_area > image_area * 0.50:
                continue

            rect = cv2.minAreaRect(contour)
            rect_width, rect_height = rect[1]

            if rect_width < 5 or rect_height < 5:
                continue

            long_side = max(rect_width, rect_height)
            short_side = min(rect_width, rect_height)
            aspect_ratio = long_side / (short_side + 1e-6)
            rectangularity = contour_area / (rect_width * rect_height + 1e-6)

            if not 1.20 <= aspect_ratio <= 2.30:
                continue
            if rectangularity < 0.40:
                continue

            center_x, center_y = rect[0]
            center_distance = math.hypot(
                (center_x - width * 0.5) / width,
                (center_y - height * 0.5) / height
            )

            score = contour_area * rectangularity * (1.0 - 0.45 * min(center_distance, 1.0))

            if score > best_score:
                best_score = score
                best_rect = rect

        if best_rect is None:
            raise RuntimeError("트레이를 검출하지 못했습니다. 트레이 전체가 영상에 보이는지 확인하세요.")

        return self._order_quad_points(cv2.boxPoints(best_rect))

    def _warp_tray(self, image, tray_quad):
        destination = np.array([
            [0, 0],
            [self.warp_width - 1, 0],
            [self.warp_width - 1, self.warp_height - 1],
            [0, self.warp_height - 1]
        ], dtype=np.float32)

        transform = cv2.getPerspectiveTransform(tray_quad, destination)
        inverse_transform = cv2.getPerspectiveTransform(destination, tray_quad)
        warped_image = cv2.warpPerspective(image, transform, (self.warp_width, self.warp_height))

        return warped_image, inverse_transform

    def _build_marker_masks(self, warped_image):
        hsv = cv2.cvtColor(warped_image, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(
            hsv,
            np.array([12, 30, 75], dtype=np.uint8),
            np.array([58, 255, 255], dtype=np.uint8)
        )

        gray = cv2.cvtColor(warped_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)

        kernel_size = max(21, (min(warped_image.shape[:2]) // 18) | 1)
        top_hat_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        top_hat = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, top_hat_kernel)

        top_hat_threshold = max(18, int(np.percentile(top_hat, 97.8)))
        gray_threshold = np.percentile(gray, 58)

        bright_mask = ((top_hat >= top_hat_threshold) & (gray >= gray_threshold)).astype(np.uint8) * 255
        combined_mask = cv2.bitwise_or(yellow_mask, bright_mask)

        combined_mask = cv2.morphologyEx(
            combined_mask, cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        )

        combined_mask = cv2.morphologyEx(
            combined_mask, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        )

        return yellow_mask, top_hat, combined_mask

    def _detect_grid_objects(self, yellow_mask, top_hat, combined_mask):
        detections = []
        object_id = 1

        for row_index, anchor_y in enumerate(self.anchor_y):
            for column_index, anchor_x in enumerate(self.anchor_x):
                expected_x = float(anchor_x * self.warp_width)
                expected_y = float(anchor_y * self.warp_height)

                marker = self._select_marker_in_cell(
                    combined_mask, yellow_mask, top_hat, expected_x, expected_y
                )

                detections.append({
                    "id": object_id,
                    "row": row_index + 1,
                    "column": column_index + 1,
                    "detected": marker["detected"],
                    "confidence": marker["confidence"],
                    "warped_x": marker["x"],
                    "warped_y": marker["y"],
                    "expected_warped_x": expected_x,
                    "expected_warped_y": expected_y,
                    "yellow_ratio": marker["yellow_ratio"],
                    "top_hat_score": marker["top_hat_score"],
                    "grid_corrected": False,
                    "correction_reason": ""
                })

                object_id += 1

        return detections

    def _select_marker_in_cell(self, combined_mask, yellow_mask, top_hat, expected_x, expected_y):
        height, width = combined_mask.shape
        search_radius_x = int(width * 0.085)
        search_radius_y = int(height * 0.105)

        x1 = max(0, int(expected_x - search_radius_x))
        y1 = max(0, int(expected_y - search_radius_y))
        x2 = min(width, int(expected_x + search_radius_x + 1))
        y2 = min(height, int(expected_y + search_radius_y + 1))

        roi = combined_mask[y1:y2, x1:x2]
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        minimum_area = width * height * 0.00005
        maximum_area = width * height * 0.008
        target_area = width * height * 0.00075

        best_candidate = None

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < minimum_area or area > maximum_area:
                continue

            perimeter = cv2.arcLength(contour, True)

            if perimeter <= 0:
                continue

            _, _, box_width, box_height = cv2.boundingRect(contour)
            aspect_ratio = box_width / (box_height + 1e-6)

            if not 0.35 <= aspect_ratio <= 2.80:
                continue

            moments = cv2.moments(contour)

            if abs(moments["m00"]) < 1e-6:
                continue

            center_x = x1 + moments["m10"] / moments["m00"]
            center_y = y1 + moments["m01"] / moments["m00"]

            normalized_distance = math.hypot(
                (center_x - expected_x) / search_radius_x,
                (center_y - expected_y) / search_radius_y
            )

            if normalized_distance > 1.0:
                continue

            contour_mask = np.zeros_like(roi)
            cv2.drawContours(contour_mask, [contour], -1, 255, -1)

            yellow_roi = yellow_mask[y1:y2, x1:x2]
            top_hat_roi = top_hat[y1:y2, x1:x2]

            yellow_ratio = cv2.mean(yellow_roi, mask=contour_mask)[0] / 255.0
            top_hat_score = cv2.mean(top_hat_roi, mask=contour_mask)[0] / 255.0
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)

            area_score = math.exp(
                -abs(math.log((area + 1.0) / (target_area + 1.0)))
            )

            # 노란색이 거의 없는 밝은 반사점은 예상 위치에서 가까울 때만 허용
            if yellow_ratio < 0.05 and normalized_distance > 0.40:
                continue

            score = (
                2.8 * yellow_ratio
                + 1.0 * min(top_hat_score * 4.0, 1.0)
                + 0.7 * max(circularity, 0.0)
                + 0.4 * area_score
                - 1.8 * normalized_distance
            )

            if best_candidate is None or score > best_candidate["score"]:
                best_candidate = {
                    "x": float(center_x),
                    "y": float(center_y),
                    "score": float(score),
                    "yellow_ratio": float(yellow_ratio),
                    "top_hat_score": float(top_hat_score)
                }

        if best_candidate is None or best_candidate["score"] < 0.35:
            return {
                "x": expected_x,
                "y": expected_y,
                "detected": False,
                "confidence": 0.0,
                "yellow_ratio": 0.0,
                "top_hat_score": 0.0
            }

        confidence = float(np.clip((best_candidate["score"] - 0.35) / 2.5, 0.0, 1.0))

        return {
            "x": best_candidate["x"],
            "y": best_candidate["y"],
            "detected": True,
            "confidence": confidence,
            "yellow_ratio": best_candidate["yellow_ratio"],
            "top_hat_score": best_candidate["top_hat_score"]
        }

    def _regularize_grid(self, detections):
        """3×4 고정 배열 구조를 이용해 반사광 오검출을 제거하고 누락된 슬롯 위치를 복원한다."""
        column_centers = {}
        row_centers = {}

        for column in range(1, 5):
            values = [
                item["warped_x"]
                for item in detections
                if item["column"] == column and item["detected"]
            ]

            column_centers[column] = (
                float(np.median(values))
                if values
                else float(self.anchor_x[column - 1] * self.warp_width)
            )

        for row in range(1, 4):
            values = [
                item["warped_y"]
                for item in detections
                if item["row"] == row and item["detected"]
            ]

            row_centers[row] = (
                float(np.median(values))
                if values
                else float(self.anchor_y[row - 1] * self.warp_height)
            )

        # 행/열 중심에서 너무 벗어난 직접 검출 제거
        for item in detections:
            if not item["detected"]:
                continue

            deviation_x = abs(item["warped_x"] - column_centers[item["column"]])
            deviation_y = abs(item["warped_y"] - row_centers[item["row"]])

            if deviation_x > self.max_column_deviation or deviation_y > self.max_row_deviation:
                item["detected"] = False
                item["confidence"] = 0.0
                item["grid_corrected"] = True
                item["correction_reason"] = "grid_outlier"

        # 오검출 제거 후 행/열 중심 재계산
        for column in range(1, 5):
            values = [
                item["warped_x"]
                for item in detections
                if item["column"] == column and item["detected"]
            ]

            if values:
                column_centers[column] = float(np.median(values))

        for row in range(1, 4):
            values = [
                item["warped_y"]
                for item in detections
                if item["row"] == row and item["detected"]
            ]

            if values:
                row_centers[row] = float(np.median(values))

        # 검출 누락/오검출 슬롯은 행·열 중심으로 복원
        for item in detections:
            if item["detected"]:
                continue

            item["warped_x"] = column_centers[item["column"]]
            item["warped_y"] = row_centers[item["row"]]

            if not item["correction_reason"]:
                item["grid_corrected"] = True
                item["correction_reason"] = "marker_missing"

        return detections

    @staticmethod
    def _transform_point(point, matrix):
        source = np.array([[[point[0], point[1]]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(source, matrix)[0, 0]
        return float(transformed[0]), float(transformed[1])

    @staticmethod
    def _transform_polygon(points, matrix):
        source = np.asarray(points, dtype=np.float32).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(source, matrix).reshape(-1, 2)

    def _draw_result(self, image, detections, tray_quad, inverse_transform):
        result = image.copy()

        cv2.polylines(
            result, [tray_quad.astype(np.int32)], True,
            (255, 0, 255), 2, cv2.LINE_AA
        )

        box_left = self.warp_width * 0.085
        box_right = self.warp_width * 0.095
        box_top = self.warp_height * 0.075
        box_bottom = self.warp_height * 0.230

        direct_count = 0
        inferred_count = 0

        for detection in detections:
            if detection["detected"]:
                color = (0, 255, 0)
                direct_count += 1
            else:
                color = (0, 165, 255)
                inferred_count += 1

            center_x = detection["warped_x"]
            center_y = detection["warped_y"]

            warped_box = [
                (center_x - box_left, center_y - box_top),
                (center_x + box_right, center_y - box_top),
                (center_x + box_right, center_y + box_bottom),
                (center_x - box_left, center_y + box_bottom)
            ]

            original_box = self._transform_polygon(warped_box, inverse_transform).astype(np.int32)

            cv2.polylines(result, [original_box], True, color, 2, cv2.LINE_AA)

            image_x = int(round(detection["image_x"]))
            image_y = int(round(detection["image_y"]))

            cv2.circle(result, (image_x, image_y), 5, color, -1, cv2.LINE_AA)
            cv2.putText(
                result,
                str(detection["id"]),
                (image_x + 7, image_y - 7),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                color,
                1,
                cv2.LINE_AA
            )

        status_text = f"Objects: {direct_count}/12  direct: {direct_count}  inferred: {inferred_count}"

        cv2.putText(
            result,
            status_text,
            (15, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
            cv2.LINE_AA
        )

        return result