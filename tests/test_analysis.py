from backend.analysis import check_sample
from backend.sources import DemoSource, make_box


def test_clean_demo_frame_has_no_flags():
    d = DemoSource()
    assert check_sample(d.boxes("demo-3"), d.points("demo-3")) == []


def test_bad_boxes_are_flagged():
    d = DemoSource()
    boxes, pts = d.boxes("demo-3"), d.points("demo-3")
    boxes.append(make_box("big", "human.pedestrian.adult", (30, 12, 0), (4, 4, 4), 0, 0, "1"))   # huge, empty
    boxes.append(make_box("fly", "vehicle.car", (14, 0, 5), (1.9, 4.5, 1.6), 0, 80, "4"))        # floating
    ids = {f["id"]: f for f in check_sample(boxes, pts)}
    assert "big" in ids and "fly" in ids and ids["big"]["severity"] == "high"


if __name__ == "__main__":
    test_clean_demo_frame_has_no_flags(); test_bad_boxes_are_flagged(); print("tests ok")
