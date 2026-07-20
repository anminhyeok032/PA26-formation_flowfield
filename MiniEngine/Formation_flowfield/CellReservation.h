#pragma once
#include <unordered_map>
#include <cstdint>

// 각 복셀마다 현재 가려고 하는 npc가 있는지 체크용
// TODO : 잦은 할당 및 접근이 이뤄지므로 추후 병목이라 판단되면 체이닝->open addressing으로 개선

class CellReservation
{
public:
    static constexpr int kEmpty = -1;

    void Reset(size_t npcCount)
    {
        m_Map.clear();
        m_Map.reserve(npcCount);   // 재해시 스파이크 방지용 사전 예약은 유지
    }

    bool TryReserve(int64_t key, int npc)
    {
        //auto [it, inserted] = m_Map.try_emplace(key, npc);
        auto inserted = m_Map.try_emplace(key, npc);
        //return inserted || it->second == npc;
        return inserted.second || inserted.first->second == npc;
    }

    void Release(int64_t key) { m_Map.erase(key); }

    int Find(int64_t key) const
    {
        auto it = m_Map.find(key);
        return it != m_Map.end() ? it->second : kEmpty;
    }

private:
    std::unordered_map<int64_t, int> m_Map;
};
