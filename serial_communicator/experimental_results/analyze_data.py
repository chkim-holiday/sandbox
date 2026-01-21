import pandas as pd
import numpy as np
import sys

# 파일명 인자 처리
if len(sys.argv) > 1:
    filename = sys.argv[1]
else:
    # filename = 'digital.csv'  # 기본값
    filename = 'digital_no_verbose_921600.csv'  # 기본값

df = pd.read_csv(filename)

ch0 = df['Channel 0'].values
times = df['Time [s]'].values

# timestamp 3-2, 5-4, 7-6 ... 처럼 홀수번째-바로 앞 짝수번째의 차이만을 모두 구해 통계(평균, 표준편차, 최소, 최대, 메디안)를 냄

even_idx = np.arange(1, len(times), 2)  # 1,3,5,...
odd_idx = even_idx - 1                  # 0,2,4,...

if len(even_idx) > 0 and np.all(even_idx < len(times)):
    intervals = (times[even_idx] - times[odd_idx]) * 1000  # ms
    print(f"짝수번째-바로 앞 홀수번째 timestamp 차이 (ms): {intervals}")
    print("\n통계 (ms):")
    print(f"  평균: {np.mean(intervals)} ms")
    print(f"  메디안: {np.median(intervals)} ms")
    print(f"  표준편차: {np.std(intervals)} ms")
    print(f"  최소: {np.min(intervals)} ms")
    print(f"  최대: {np.max(intervals)} ms")
else:
    print("데이터가 부족하거나 인덱스 오류로 통계 불가.")