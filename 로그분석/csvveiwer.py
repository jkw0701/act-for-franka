import pandas as pd
import matplotlib.pyplot as plt
import re

# CSV 파일 로딩 (세미콜론으로 구분)
df = pd.read_csv("5-2.7만번 학습,선행 앙상블 100회 로그2.csv", sep=';', encoding='utf-8')

# message 컬럼에서 성공/실패 여부, x, y 좌표 추출
pattern = r'(성공|실패)한 박스 위치: x=([-+]?\d*\.\d+), y=([-+]?\d*\.\d+)'
parsed = df['message'].str.extract(pattern)

# 컬럼 이름 지정 및 타입 변환
parsed.columns = ['status', 'x', 'y']
parsed['x'] = parsed['x'].astype(float)
parsed['y'] = parsed['y'].astype(float)

# 성공 확률 계산
success_rate = (parsed['status'] == '성공').mean()
#print(f"✅ 성공 확률: {success_rate:.2%}")

# 시각화
plt.figure(figsize=(8, 6))

# 'success' 점 (파란색)
plt.scatter(parsed[parsed['status'] == '성공']['x'],
            parsed[parsed['status'] == '성공']['y'],
            color='blue', label='Success', alpha=0.7)

# 'fail' 점 (빨간색)
plt.scatter(parsed[parsed['status'] == '실패']['x'],
            parsed[parsed['status'] == '실패']['y'],
            color='red', label='Fail', alpha=0.7)

plt.xlabel('X', fontsize=14)
plt.ylabel('Y', fontsize=14)
plt.title(f'success %: {success_rate:.2%}', fontsize=16)
plt.legend(fontsize=12)
plt.grid(True)
plt.tight_layout()
plt.show()