#include "HeightMap.h"
#include <cstdio>
#include <cmath>
#include <algorithm>

bool HeightMap::LoadFromBMP(const std::string& filePath, float maxHeight, float worldScale, float voxelSize)
{
    FILE* fp = nullptr;
    fopen_s(&fp, filePath.c_str(), "rb");
    if (!fp) return false;

    // BMP 파일 헤더 14바이트
    uint8_t fileHeader[14] = {};
    fread(fileHeader, 1, 14, fp);

    // 시그니처 확인 'BM'
    if (fileHeader[0] != 'B' || fileHeader[1] != 'M')
    {
        fclose(fp);
        return false;
    }

    // 픽셀 데이터 시작 오프셋
    uint32_t pixelOffset = *reinterpret_cast<uint32_t*>(&fileHeader[10]);

    // DIB 헤더 40바이트 (BITMAPINFOHEADER)
    uint8_t dibHeader[40] = {};
    fread(dibHeader, 1, 40, fp);

    int      width = *reinterpret_cast<int32_t*> (&dibHeader[4]);
    int      depth = std::abs(*reinterpret_cast<int32_t*>(&dibHeader[8]));
    uint16_t bpp = *reinterpret_cast<uint16_t*>(&dibHeader[14]);

    // 24bit / 8bit BMP만 지원
    if (bpp != 24 && bpp != 8)
    {
        fclose(fp);
        return false;
    }

    m_Width = width;
    m_Depth = depth;
    m_MaxHeight = maxHeight;
    m_WorldScale = worldScale;
    m_VoxelSize = voxelSize;

    m_Heights.resize(width * depth);

    // 픽셀 데이터로 이동
    fseek(fp, pixelOffset, SEEK_SET);

    // BMP는 각 행이 4바이트 배수로 패딩됨
    int bytesPerPixel = bpp / 8;
    int rowStride = (width * bytesPerPixel + 3) & ~3;
    std::vector<uint8_t> row(rowStride);

    // BMP는 아래 행부터 저장되므로 역순으로 읽기
    for (int z = depth - 1; z >= 0; z--)
    {
        fread(row.data(), 1, rowStride, fp);
        for (int x = 0; x < width; x++)
        {
            uint8_t grayValue = 0;
            if (bpp == 24)
                grayValue = row[x * 3 + 2]; // BGR 순서 → R채널
            else
                grayValue = row[x];          // 8bit grayscale

            // 0~255 → 0.0~maxHeight
            m_Heights[Index(x, z)] = (grayValue / 255.0f) * maxHeight;
        }
    }

    fclose(fp);
    return true;
}

float HeightMap::GetHeight(int x, int z) const
{
    // 범위 클램프
    x = std::max(0, std::min(x, m_Width - 1));
    z = std::max(0, std::min(z, m_Depth - 1));
    return m_Heights[Index(x, z)];
}

// 월드 좌표 → 바이리니어 보간으로 높이 샘플링
float HeightMap::SampleHeight(float worldX, float worldZ) const
{
    // 월드 좌표 → 픽셀 좌표 (실수)
    float px = worldX / m_WorldScale;
    float pz = worldZ / m_WorldScale;

    // 정수 픽셀 인덱스
    int x0 = (int)px;
    int z0 = (int)pz;
    int x1 = x0 + 1;
    int z1 = z0 + 1;

    // 범위 클램프
    x0 = std::max(0, std::min(x0, m_Width - 1));
    z0 = std::max(0, std::min(z0, m_Depth - 1));
    x1 = std::max(0, std::min(x1, m_Width - 1));
    z1 = std::max(0, std::min(z1, m_Depth - 1));

    // 소수점 부분 (보간 비율)
    float tx = px - (float)(int)px;
    float tz = pz - (float)(int)pz;

    // 4개 픽셀 높이
    float h00 = m_Heights[Index(x0, z0)];
    float h10 = m_Heights[Index(x1, z0)];
    float h01 = m_Heights[Index(x0, z1)];
    float h11 = m_Heights[Index(x1, z1)];

    // 바이리니어 보간
    float h0 = h00 + tx * (h10 - h00);
    float h1 = h01 + tx * (h11 - h01);
    return h0 + tz * (h1 - h0);
}