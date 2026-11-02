"""Optional on-device smoke test (requires pytest-embedded-idf and an ESP32-S3)."""
import pytest
from pytest_embedded_idf.dut import IdfDut

@pytest.mark.esp32s3
def test_n10_bridge_boot(dut: IdfDut) -> None:
    dut.expect("RobotCar-ESP32S3; App TCP3333, PC TCP3335, N10 UDP3334; RX16 TX15 / RX18 TX17")
    dut.expect("valid=", timeout=10)

