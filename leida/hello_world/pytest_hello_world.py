"""Optional on-device smoke test (requires pytest-embedded-idf and an ESP32-S3)."""
import pytest
from pytest_embedded_idf.dut import IdfDut

@pytest.mark.esp32s3
def test_n10_bridge_boot(dut: IdfDut) -> None:
    dut.expect("N10-LiDAR / n10lidar123; UDP 192.168.4.1:3334; UART2 RX16 TX15")
    dut.expect("valid=", timeout=10)

